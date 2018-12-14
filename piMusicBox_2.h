/*
 * piMusicBox_1.h
 *
 *  Created on: 11 de sep. de 2017
 *      Author: FFM
 */

#ifndef PIMUSICBOX_2_H_
#define PIMUSICBOX_2_H_

#include <stdlib.h> // para poder usar NULL
#include <stdio.h> // para poder usar printf
#include <stdlib.h>
#include <wiringPi.h>
#include <string.h>
#include <time.h>
#include <signal.h>
#include <errno.h>
#include <unistd.h>
#include <sys/time.h>


#include "kbhit.h" // Para poder detectar teclas pulsadas sin bloqueo y leer las teclas pulsadas
#include "fsm.h" // para poder crear y ejecutar la máquina de estados

#define CLK_MS 10 // PERIODO DE ACTUALIZACION DE LA MAQUINA ESTADOS

// FLAGS DEL SISTEMA
#define FLAG_PLAYER_START 			0x01
#define FLAG_PLAYER_END				0x02
#define FLAG_NOTA_TIMEOUT			0x04
#define FLAG_PLAYER_STOP			0x08
#define FLAG_CARD_IN				0X10
#define FLAG_VALID_CARD				0X20
#define FLAG_SYSTEM_START			0X40
#define FLAG_SYSTEM_END				0X80

// A 'key' which we can lock and unlock - values are 0 through 3
//	This is interpreted internally as a pthread_mutex by wiringPi
//	which is hiding some of that to make life simple.
#define	FLAGS_KEY	1
#define	STD_IO_BUFFER_KEY	2



extern int frecuenciaDespacito[];
extern int tiempoDespacito[];
extern int frecuenciaGOT[];
extern int tiempoGOT[];
extern int frecuenciaTetris[];
extern int tiempoTetris[];
extern int frecuenciaStarwars[];
extern int tiempoStarwars[];

//DEFINICION DE LOS ESTADOS
typedef enum {
	WAIT_START,
	WAIT_NEXT,
	WAIT_END,
	WAIT_CHECK,
	WAIT_PLAY,
	WAIT_CARD} TipoEstadosSistema;

#define MAX_NUM_TARJETAS	5
#define MAX_NUM_NOTAS 		600
#define MAX_NUM_CHAR_NOMBRE	100
#define MAX_NUM_CHAR_UID	100

typedef struct {
	char nombre[MAX_NUM_CHAR_NOMBRE]; // String con el nombre de la melodia
	int frecuencias[MAX_NUM_NOTAS]; // Array con las frecuencias de las notas de la melodia
	int duraciones[MAX_NUM_NOTAS]; // Array con las duraciones de las notas de la melodia
	int num_notas; // Numero de notas de que consta la melodia
} TipoMelodia;


typedef struct {
	int posicion_nota_actual;	// Valor correspondiente a la posicion de la nota actual en los arrays de frecuencias y duraciones
	int frecuencia_nota_actual; // Valor correspondiente a la frecuencia de la nota actual
	int duracion_nota_actual; // Valor correspondiente a la duracion de la nota actual

	TipoMelodia* melodia;

	// A completar por el alumno
	// ...
} TipoPlayer;

typedef struct {
	//Uid uid;  // Identificador correspondiente a la tarjeta (tipo de datos complejo definido en liberia control RFID)
	TipoMelodia melodia;
} TipoTarjeta;

typedef struct {
	TipoPlayer player; // Reproductor de melodias

	int num_tarjetas_activas; // Numero de tarjetas validas

	TipoTarjeta tarjetas_activas[MAX_NUM_TARJETAS]; // Array con todas las tarjetas validas

	int pos_tarjeta_actual; // Posicion de la tarjeta actual en el array de tarjetas validas

	char* uid_tarjeta_actual_string; // Identificador de la tarjeta actual a modo de string de caracteres

	TipoEstadosSistema estado; // Variable que almacena el estado actual del sistema

	char teclaPulsada; // Variable que almacena la ultima tecla pulsada

	int debug; // Variable que habilita o deshabilita la impresion de mensajes por salida estandar
} TipoSistema;

//------------------------------------------------------
// FUNCIONES DE ENTRADA O DE TRANSICION DE LA MAQUINA DE ESTADOS
//------------------------------------------------------
// Prototipos de funciones de entrada
int CompruebaPlayerStart (fsm_t* this);
int CompruebaFinalMelodia (fsm_t* this);
int CompruebaNotaTimeout (fsm_t* this);
int CompruebaNuevaNota (fsm_t* this);
int CompruebaPlayerStop (fsm_t* this);

int CompruebaComienzo(fsm_t*);
int TarjetaNoDisponible(fsm_t*);
int TarjetaDisponible(fsm_t*);
int TarjetaNoValida(fsm_t*);
int TarjetaValida(fsm_t*);
int CompruebaFinalReproduccion(fsm_t*);



//------------------------------------------------------
// FUNCIONES DE SALIDA O DE ACCION DE LA MAQUINA DE ESTADOS
//------------------------------------------------------
//void InicializaMelodia(fsm_t*);
void InicializaPlayer(fsm_t*);
void ActualizaPlayer(fsm_t*);
void StopPlayer(fsm_t*);
void FinalMelodia(fsm_t*);
void ComienzaNuevaNota(fsm_t*);
int InicializaMelodia (TipoMelodia *melodia, char *nombre, int *array_frecuencias, int *array_duraciones, int num_notas);


void ComienzaSistema(fsm_t*);
void EsperoTarjeta(fsm_t*);
void LeerTarjeta(fsm_t*);
void DescartaTarjeta(fsm_t*);
void ComienzaReproduccion(fsm_t*);
void CancelaReproduccion(fsm_t*);
void ComprueboTarjeta(fsm_t*);
void FinalizaReproduccion(fsm_t*);

int systemSetup (void);
PI_THREAD (thread_explora_teclado);
void detecta_tarjeta();

#endif /* PIMUSICBOX_2_H_ */
