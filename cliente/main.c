/* CLIENT */
#include <stdio.h> //printf
#include <stdlib.h>
#include <string.h>     //strlen
#include <sys/socket.h> //socket
#include <arpa/inet.h>  //inet_addr

#include <time.h>
#include "structsBalistica.h"
#include <stdbool.h>
#include <math.h>

#include <unistd.h>
#include <pthread.h>
#include <semaphore.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define ipAconectar "127.0.0.1"
#define porta 5007

#define qteDisparospossiveis 4

#define xAlvo 10000.0
#define yAlvo 10000.0
#define zAlvo 0.0

#define xCanhao 10000.0
#define yCanhao 10000.0
#define zCanhao 0.0
//#define velProjetil 694.444444
// todas as unidades de medidas estao no S.I.

int sock;
struct sockaddr_in server;

double tempoInicial;
double tempoFinal;
Objeto projetil[qteDisparospossiveis];

double instanteDisparo[qteDisparospossiveis];
double instanteInicialDisparo[qteDisparospossiveis];
int tentativasDisparo;

pthread_mutex_t client_mutex = PTHREAD_MUTEX_INITIALIZER;
sem_t semaforo;

int abreComunicacao();
int quadranteRelativoXY(Posicao *referencia, Posicao *objeto);
void setDirecao(Objeto* voador, Posicao* alvo, double speed);
void *threadRecebeResposta (void *p);
void printPosicao(Posicao *pos);
void printVelocidade(Vetor *vel);


int main(int argc , char *argv[]){
	srand(time(NULL));

    double raioRadar = 10000.0; //10k metros
    Posicao centro;
    centro.x = xAlvo;
    centro.y = yAlvo;
    centro.z = zAlvo;
    
    Objeto aviao;
    // posicao aleatoria inicial do aviao eh nas bordas do do radar (quando ele eh detectado)
    // (x - alvo.x)^2 + (y - alvo.y)^2 = raio^2
    // sorteia um valor aleatorio para posicao x
    aviao.pos.x =  (double) (rand()%((int) raioRadar*2 +1));

    // tendo 1 valor x, ha 2 possiveis valores para y, 1 deles eh sorteado
    if (rand()%2 ){
        aviao.pos.y =  sqrt( pow(raioRadar, 2) - pow( (aviao.pos.x - centro.x) , 2) ) + centro.y;
    }
    else{
        aviao.pos.y =  -sqrt( pow(raioRadar, 2) - pow( (aviao.pos.x - centro.x) , 2) ) + centro.y;
    }
    
    aviao.pos.z = 500.0; //500 metros, altura constante
    double velocidadeAviao = (double) ( (rand()%101 + 300)/3.6) ; //V aleatoria entre 300-400 km/h, convertida para m/s
    aviao.voando = true;
    
    setDirecao(&aviao, &centro ,velocidadeAviao); //direciona o aviao ao alvo, no caso o proprio radar
    printf("quadrante %d\n", quadranteRelativoXY(&aviao.pos, &centro));
    fflush(stdout);
    printVelocidade(&aviao.velocidade);
    
    

    printf("velocidade vetorial: %0.1f\n", velocidadeAviao);
    fflush(stdout);
    
    if (abreComunicacao() == 1) {
        puts("Falhou em abrir a conexao. \n");
        return 1;
    }
    
    tentativasDisparo = 0;
    pthread_t processaResposta;
    pthread_create(&processaResposta, NULL, threadRecebeResposta, (void *)&aviao);
    
    tempoInicial = 0.0;
    tempoFinal = 0.0;
    int centesimos = 0;
    //keep communicating with server
    do{
        if ( centesimos%50 == 0 ) {
            if( send(sock , (void*)&aviao.pos ,sizeof(Posicao), 0) < 0){
                puts("Send failed");
                return 1;
            }
            usleep(500000); //sleep de meio segundo
        }
        
        centesimos++;
        tempoFinal = 0.01*centesimos;
        
        pthread_mutex_lock(&client_mutex);
        aviao.pos.x = aviao.pos.x + aviao.velocidade.x* ( tempoFinal - tempoInicial ) ;
        aviao.pos.y = aviao.pos.y + aviao.velocidade.y* ( tempoFinal - tempoInicial ) ;
        aviao.pos.z = aviao.pos.z + aviao.velocidade.z* ( tempoFinal - tempoInicial ) ;
        
        for (int i = 0; i < tentativasDisparo;  i++) {
            if (tempoFinal > instanteDisparo[i]) {
                printf("projetil %d ", i);
                printPosicao(&projetil[i].pos);
            }
            
            while (tempoFinal > instanteDisparo[i]) {
                projetil[i].pos.x = projetil[i].pos.x + projetil[i].velocidade.x*(0.01);
                projetil[i].pos.y = projetil[i].pos.y + projetil[i].velocidade.y*(0.01);
                projetil[i].pos.z = zCanhao +
                                    projetil[i].velocidade.z*(instanteDisparo[i] - instanteInicialDisparo[i]) +
                                    (-1.0)*(gravidade*pow((instanteDisparo[i] - instanteInicialDisparo[i]), 2))/2.0;
                instanteDisparo[i] = instanteDisparo[i]+0.01;
                
//                printf("tf: %0.1f    tinicial: %0.1f", instanteDisparo[i], instanteInicialDisparo[i]);
//                fflush(stdout);
            }
        }
        pthread_mutex_unlock(&client_mutex);
        
        tempoInicial = tempoFinal;
        
        pthread_mutex_lock(&client_mutex);
        for (int i = 0; i < tentativasDisparo; i++) {
            if ( pow( projetil[i].pos.x - aviao.pos.x , 2) +
                 pow( projetil[i].pos.y - aviao.pos.y , 2) +
                 pow( projetil[i].pos.z - aviao.pos.z , 2) <= pow(5.0, 2) ) {
                printf("Kaboom! voce foi atingido! derrubado no instante: %0.1f \n", tempoFinal); //acertou o aviao
                fflush(stdout);
                aviao.voando = false; 
            }
        }
        pthread_mutex_unlock(&client_mutex);
        
        if ( pow( aviao.pos.x - centro.x , 2) +
             pow( aviao.pos.y - centro.y , 2) /*+
             pow( aviao.pos.z - centro.z , 2) */<= pow(500, 2) ) {
            
            puts("the bomb has been planted.");
                 
            puts("Alvo alcancado! pronto para disparar!");
            printf("posicao x: %0.1f posicao y: %0.1f posicao z: %0.1f \n", aviao.pos.x, aviao.pos.y, aviao.pos.z);
            fflush(stdout);
            pthread_mutex_lock(&client_mutex);
            aviao.voando = false;
            pthread_mutex_unlock(&client_mutex);
        }

    }while(aviao.voando);
    
    //pthread_join(processaResposta, NULL); //nao incluir no codigo pois causa deadlock e nao termina a execucao
    close(sock);
    return 0;
}

void *threadRecebeResposta (void *p){
    //    pthread_mutex_lock(&client_mutex);
    //    sem_post(&semaforo);
    //    sem_wait(&semaforo);
    //    pthread_mutex_unlock(&client_mutex);
    Objeto *aviao = (Objeto *) p;
    Info disparo;

    do{
        //Receive a reply from the server
        if( recv(sock , (void*)&disparo , sizeof(disparo) , 0) < 0){
            pthread_mutex_lock(&client_mutex);
            puts("recv failed");
            aviao->voando = false;
            pthread_mutex_unlock(&client_mutex);
            break;
        }
        
        if (disparo.elevacao != 0 && disparo.azinute != 0 && tentativasDisparo < qteDisparospossiveis) {
            printf("disparo %d sera efetuado aos %0.1f!\n", tentativasDisparo, disparo.instante);
            fflush(stdout);
            
            pthread_mutex_lock(&client_mutex);
            projetil[tentativasDisparo].pos.x = xCanhao;
            projetil[tentativasDisparo].pos.y = yCanhao;
            projetil[tentativasDisparo].pos.z = zCanhao;
            
            double velXY = disparo.velDisparo*cos(disparo.elevacao);
            setDirecao(&projetil[tentativasDisparo], &aviao->pos, velXY);
            projetil[tentativasDisparo].velocidade.z = disparo.velDisparo* sin(disparo.elevacao);
            //printVelocidade(&projetil[tentativasDisparo].velocidade);
            instanteDisparo[tentativasDisparo] = disparo.instante;
            instanteInicialDisparo[tentativasDisparo] = disparo.instante;
            
            tentativasDisparo++;
            pthread_mutex_unlock(&client_mutex);
        }
        
    }while(aviao->voando);

    pthread_exit(NULL);
}

int abreComunicacao(){
    //Create socket
    sock = socket(AF_INET , SOCK_STREAM , 0);
    if (sock == -1){
        printf("Could not create socket");
        return 1;
    }
    puts("Socket created");
    
    server.sin_addr.s_addr = inet_addr(ipAconectar);
    server.sin_family = AF_INET;
    server.sin_port = htons( porta );
    if (connect(sock , (struct sockaddr *)&server , sizeof(server)) < 0){
        perror("connect failed. Error");
        return 1;
    }
    puts("Connected\n");
    return 0;
}

void setDirecao(Objeto* voador, Posicao* alvo, double speed){
    //direcao  = angulo => (vel.x, vel.x)
    double angulo = atan( fabs(voador->pos.y - alvo->y)/ fabs(voador->pos.x - alvo->x) );
    
    //int quadrante = quadranteRelativoXY(&voador->pos, alvo);
    switch ( quadranteRelativoXY(&voador->pos, alvo) ) { //switch (quadrante){
        case 1:
            //quad 1  <- \/
            voador->velocidade.x = (double) (speed * cos(angulo)) *(-1.0); // <-
            voador->velocidade.y = (double) (speed * sin(angulo)) *(-1.0); // \/
            //printf("quad 1\n");
            break;
        case 2:
            //quad 2  -> \/
            voador->velocidade.x = (double) (speed * cos(angulo));        // ->
            voador->velocidade.y = (double) (speed * sin(angulo)) *(-1.0);// \/
            //printf("quad 2\n");
            break;
        case 3:
            //quad 3  -> ^
            voador->velocidade.x = (double) (speed * cos(angulo));        // ->
            voador->velocidade.y = (double) (speed * sin(angulo));        // ^
            //printf("quad 3\n");
            break;
        case 4:
            //quad 4  <- ^
            voador->velocidade.x = (double) (speed * cos(angulo)) *(-1.0);// <-
            voador->velocidade.y = (double) (speed * sin(angulo));        // ^
            //printf("quad 4\n");
            break;
        default:
            break;
    }
    voador->velocidade.z = 0;
}

int quadranteRelativoXY(Posicao *referencia, Posicao *objeto){
    //verifica em qual quadrante esta o aviao para definir em qual direcao o mesmo deve voar
    if (referencia->x < objeto->x) { // ta no quad 2 ou 3
        if (referencia->y < objeto->y ) {
            //quad 3
            return 3;
        }
        else{
            //quad 2
            return 2;
        }
    }
    else{ //ta no quad 1 ou 4
        if (referencia->y < objeto->y ) {
            //quad 4
            return 4;
        }
        else{
            //quad 1
            return 1;
        }
    }
}

void printPosicao(Posicao *pos){
    printf("p.x: %0.1f  p.y: %0.1f p.z: %0.1f \n", pos->x, pos->y, pos->z);
    fflush(stdout);
}

void printVelocidade(Vetor *vel){
    printf("v.x: %0.1f  v.y: %0.1f v.z: %0.1f \n", vel->x, vel->y, vel->z);
    fflush(stdout);
}

