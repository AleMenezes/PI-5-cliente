//
//  structsBalistica.h
//  servidor
//
//  Created by Alessandro Camillo Gimenez de Menezes on 30/04/14.
//  Copyright (c) 2014 Alessandro Camillo Gimenez de Menezes. All rights reserved.
//
#include <stdbool.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef gravidade
#define gravidade 9.80665
#endif

#ifndef servidor_structsBalistica_h
#define servidor_structsBalistica_h

#ifndef Posicao
typedef struct{
    double x;
    double y;
    double z;
}Posicao;
#endif

#ifndef Vetor
typedef struct{
    double x;
    double y;
    double z;
}Vetor;
#endif

#ifndef Info
typedef struct{
    double elevacao; //anguloChao;
    double azinute;  //anguloNorte;
    double velDisparo;
    double instante;
}Info;
#endif

#ifndef Objeto
typedef struct{
    Posicao pos;
    Vetor velocidade;
    bool voando;
}Objeto;
#endif


#endif
