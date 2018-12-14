
#include "piMusicBox_2.h"
#include "fsm.h"
#include "tmr.h"
#include <time.h>
#include <sys/time.h>
#include "mfrc522.h"
#include "rfid.h"
#include <wiringPi.h>
#include <softTone.h>
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include  <unistd.h>


int frecuenciaDespacito[160] = {0,1175,1109,988,740,740,740,740,740,740,988,988,988,988,880,988,784,0,784,784,784,784,784,988,988,988,988,1109,1175,880,0,880,880,880,880,880,1175,1175,1175,1175,1318,1318,1109,0,1175,1109,988,740,740,740,740,740,740,988,988,988,988,880,988,784,0,784,784,784,784,784,988,988,988,988,1109,1175,880,0,880,880,880,880,880,1175,1175,1175,1175,1318,1318,1109,0,1480,1318,1480,1318,1480,1318,1480,1318,1480,1318,1480,1568,1568,1175,0,1175,1568,1568,1568,0,1568,1760,1568,1480,0,1480,1480,1480,1760,1568,1480,1318,659,659,659,659,659,659,659,659,554,587,1480,1318,1480,1318,1480,1318,1480,1318,1480,1318,1480,1568,1568,1175,0,1175,1568,1568,1568,1568,1760,1568,1480,0,1480,1480,1480,1760,1568,1480,1318};
int tiempoDespacito[160] = {1200,600,600,300,300,150,150,150,150,150,150,150,150,300,150,300,343,112,150,150,150,150,150,150,150,150,300,150,300,300,150,150,150,150,150,150,150,150,150,300,150,300,800,300,600,600,300,300,150,150,150,150,150,150,150,150,300,150,300,343,112,150,150,150,150,150,150,150,150,300,150,300,300,150,150,150,150,150,150,150,150,150,300,150,300,450,1800,150,150,150,150,300,150,300,150,150,150,300,150,300,450,450,300,150,150,225,75,150,150,300,450,800,150,150,300,150,150,300,450,150,150,150,150,150,150,150,150,300,300,150,150,150,150,150,150,450,150,150,150,300,150,300,450,450,300,150,150,150,300,150,300,450,800,150,150,300,150,150,300,450};
int frecuenciaGOT[518] = {1568,0,1046,0,1244,0,1397,0,1568,0,1046,0,1244,0,1397,0,1175,0,1397,0,932,0,1244,0,1175,0,1397,0,932,0,1244,0,1175,0,1046,0,831,0,698,0,523,0,349,0,784,0,523,0,523,0,587,0,622,0,698,0,784,0,523,0,622,0,698,0,784,0,523,0,622,0,698,0,587,0,698,0,466,0,622,0,587,0,698,0,466,0,622,0,587,0,523,0,523,0,587,0,622,0,698,0,784,0,523,0,622,0,698,0,784,0,523,0,622,0,698,0,587,0,698,0,466,0,622,0,587,0,698,0,466,0,622,0,587,0,523,0,0,1568,0,0,1046,0,0,1244,0,0,1397,0,0,1568,0,0,1046,0,0,1244,0,0,1397,0,0,1175,0,587,0,622,0,587,0,523,0,587,0,784,0,880,0,932,0,1046,0,1175,0,0,1397,0,0,932,0,0,1244,0,0,1175,0,0,1397,0,0,932,0,0,1244,0,0,1175,0,0,1046,0,0,1568,0,0,1046,0,0,1244,0,0,1397,0,0,1568,0,0,1046,0,0,1244,0,0,1397,0,0,1175,0,880,0,784,0,932,0,1244,0,0,1397,0,0,932,0,0,1175,0,0,1244,0,0,1175,0,0,932,0,0,1046,0,0,2093,0,622,0,831,0,932,0,1046,0,622,0,831,0,1046,0,0,1865,0,622,0,784,0,831,0,932,0,622,0,784,0,932,0,0,1661,0,523,0,698,0,784,0,831,0,523,0,698,0,831,0,0,1568,0,1046,0,1244,0,1397,0,1568,0,1046,0,1244,0,1397,0,0,0,1661,0,1046,0,1175,0,1244,0,831,0,1175,0,1244,0,0,0,0,2489,0,0,0,0,2794,0,0,0,0,3136,0,0,2093,0,622,0,831,0,932,0,1046,0,622,0,831,0,1046,0,0,1865,0,622,0,784,0,831,0,932,0,622,0,784,0,932,0,0,1661,0,523,0,698,0,784,0,831,0,523,0,698,0,831,0,0,1568,0,1046,0,1244,0,1397,0,1568,0,1046,0,1244,0,1397,0,0,0,1661,0,1046,0,1175,0,1244,0,831,0,1175,0,1244,0,0,0,0,2489,0,1397,0,0,0,2350,0,0,0,2489,0,0,0,2350,0,0,0,0,2093,0,392,0,415,0,466,0,523,0,392,0,415,0,466,0,523,0,392,0,415,0,466,0,2093,0,1568,0,1661,0,1865,0,2093,0,1568,0,1661,0,1865,0,2093,0,1568,0,1661,0,1865};
int tiempoGOT[518] = {900,89,900,89,133,13,133,13,600,59,600,59,133,13,133,13,1400,1400,900,89,900,89,133,13,133,13,600,59,900,89,133,13,133,13,1200,116,267,28,267,28,267,28,900,89,900,89,1400,89,69,7,69,7,69,7,69,7,900,89,900,89,133,13,133,13,600,59,600,59,133,13,133,13,1800,1800,900,89,900,89,133,13,133,13,600,59,900,89,133,13,133,13,1200,2400,69,7,69,7,69,7,69,7,900,89,900,89,133,13,133,13,600,59,600,59,133,13,133,13,1800,1800,900,89,900,89,133,13,133,13,600,59,900,89,133,13,133,13,1200,2400,3600,900,89,900,900,89,900,133,13,150,133,13,150,600,59,600,600,59,600,133,13,150,133,13,150,1200,400,69,7,69,7,69,7,69,7,267,28,400,45,133,13,267,28,267,28,267,28,300,900,89,900,900,89,900,133,13,150,133,13,150,600,59,600,900,89,900,133,13,150,133,13,150,1200,1800,3600,900,89,900,900,89,900,133,13,150,133,13,150,600,59,600,600,59,600,133,13,150,133,13,150,1200,400,267,28,1200,400,133,13,133,13,150,900,89,900,900,89,900,600,59,600,267,28,300,600,59,600,267,28,300,1200,2400,3600,267,28,267,28,133,13,133,13,267,28,267,28,133,13,133,13,150,267,28,267,28,133,13,133,13,133,13,267,28,267,28,133,13,150,267,28,267,28,133,13,133,13,267,28,267,28,133,13,133,13,150,267,28,267,28,133,13,133,13,267,28,267,28,133,13,133,13,150,150,600,59,133,13,133,13,267,28,267,28,133,13,133,13,150,150,150,900,89,900,900,900,900,89,900,900,900,1200,2400,3600,267,28,267,28,133,13,133,13,267,28,267,28,133,13,133,13,150,267,28,267,28,133,13,133,13,267,28,267,28,133,13,133,13,150,267,28,267,28,133,13,133,13,267,28,267,28,133,13,133,13,150,267,28,267,28,133,13,133,13,267,28,267,28,133,13,133,13,150,150,600,59,133,13,133,13,267,28,267,28,133,13,133,13,150,150,150,600,212,133,13,150,150,267,28,300,300,400,45,450,450,133,13,150,150,150,267,28,267,28,133,13,133,13,267,28,267,28,133,13,133,13,267,28,267,28,133,13,2400,116,267,28,267,28,133,13,133,13,267,28,267,28,133,13,133,13,267,28,267,28,133,13,2400};
int frecuenciaTetris[55] = {1319,988,1047,1175,1047,988,880,880,1047,1319,1175,1047,988,988,1047,1175,1319,1047,880,880,0,1175,1397,1760,1568,1397,1319,1047,1319,1175,1047,988,988,1047,1175,1319,1047,880,880,0,659,523,587,494,523,440,415,659,523,587,494,523,659,880,831};
int tiempoTetris[55] = {450,225,225,450,225,225,450,225,225,450,225,225,450,225,225,450,450,450,450,450,675,450,225,450,225,225,675,225,450,225,225,450,225,225,450,450,450,450,450,450,900,900,900,900,900,900,1800,900,900,900,900,450,450,900,1800};
int frecuenciaStarwars[59] = {523,0,523,0,523,0,698,0,1046,0,0,880,0,784,0,1397,0,523,0,1760,0,0,880,0,784,0,1397,0,523,0,1760,0,0,880,0,784,0,1397,0,523,0,1760,0,0,880,0,1760,0,0,784,0,523,0,0,523,0,0,523,0};
int tiempoStarwars[59] = {134,134,134,134,134,134,536,134,536,134,134,134,134,134,134,536,134,402,134,134,429,357,134,134,134,134,536,134,402,134,134,429,357,134,134,134,134,536,134,402,134,134,429,357,134,134,134,429,357,1071,268,67,67,268,67,67,67,67,67};

/**
 * Definimos una serie de arrays que nos servir�n de almacen de los nombres, frecuencias
 * 	tiermpos, uids y n�mero de notas de cada melod�a. Los recorreremos para decidir qu�		
 * 	melod�a reproducir.
 * Tambi�n definimos una variable para meter el id de la tarjeta que hayamos le�do 
 * y que inicialmente la ponemos a "00000000".
 */

int *frecuencias[] = {frecuenciaDespacito,frecuenciaGOT,frecuenciaTetris,frecuenciaStarwars};
int *tiempos[] = {tiempoDespacito,tiempoGOT,tiempoTetris,tiempoStarwars};
char* uid[] = {"4E0ED003", "88043368", "4E92D90B", "88048152"};
char *nombres[] = {"Despacito", "GOT", "Tetris", "StarWars"};
int numNotas[] = {160,518,55,59}; 
char *UIDTarjetaLeida = "00000000";

volatile int flags = 0;

//Definimos el tiempo de rebote.
int debounceTime = 500; 


tmr_t* timer_tmr;

/**
 * @brief	 Pone a 1 el FLAG_NOTA_TIMEOUT para que salte ActualizaPlayer y reproduzca la nota.
 * @param			
 * @return			
 * @note	 Esta funci�n se ejecuta cuando el timer acaba la cuenta.
 */

static void timer_isr () { 
	piLock(FLAGS_KEY);
	flags |= FLAG_NOTA_TIMEOUT;
	piUnlock(FLAGS_KEY);
}



/*--------------------------------------------------------------------------------------
 *                        FUNCIONES DE SALIDA MAQUINA DE ESTADOS 1
 ---------------------------------------------------------------------------------------
*/

/**
 * @brief	Carga los arrays de frecuencia y duraci�n de la melodia escogida.
 * @param	*melodia                  Puntero a la melod�a
 			*nombre					  Puntero al nombre de la melod�a
 			*array_frecuencias	      Puntero al array de frecuencias
 			*array_duraciones         Puntero al array de duraciones
 			num_notas                 Numero de notas de la melod�a

 * @return	Devuelve el n�mero de notas que tiene la melod�a.
 * @note	
 */

int InicializaMelodia (TipoMelodia *melodia, char *nombre, int *array_frecuencias, int *array_duraciones, int num_notas) {

	int i = 0;
	strcpy(melodia->nombre, nombre);
	for( i =0; i< num_notas; i++){
		melodia->frecuencias[i]= array_frecuencias[i];
		melodia->duraciones[i] = array_duraciones[i];
	}

	melodia->num_notas=num_notas;

	return melodia->num_notas;
}

/**
 * @brief	Pone a 0 el FLAG_PLAYER_START. Inicializa la primera nota cargando su array de frecuencias y duraciones. 
 			Tambi�n llama al timer para que �ste reproduzca todas las dem�s y nos saca por pantalla que acabamos de empezar.
 * @param	this        Puntero al aut�mata que usa la funci�n.
 			
 * @return	
 * @note	Funci�n que se ejecuta cuando CompruebaPlayerStart devuelve 1 (por el diagrama de estados).
 */

void InicializaPlayer(fsm_t* this){ 
	    TipoPlayer *p_player;
		p_player = (TipoPlayer*)(this->user_data);

		piLock (FLAGS_KEY); 
		flags &= ~FLAG_PLAYER_START;
		piUnlock (FLAGS_KEY);

		p_player->posicion_nota_actual = 0;
		p_player->frecuencia_nota_actual = p_player->melodia->frecuencias[p_player->posicion_nota_actual];
		p_player->duracion_nota_actual = p_player->melodia->duraciones[p_player->posicion_nota_actual];
		tmr_startms( timer_tmr, p_player->duracion_nota_actual);

		piLock (STD_IO_BUFFER_KEY); 
		printf("\n[PLAYER]Empieza la reproduccion\n");
		printf("\n[PLAYER][ActualizaPlayer][NOTA %d de %d][FREC %d][DURA %d]\n",p_player->posicion_nota_actual+1, p_player->melodia->num_notas, p_player->frecuencia_nota_actual,p_player->duracion_nota_actual);

		piUnlock (STD_IO_BUFFER_KEY);
		softToneWrite(18,p_player->frecuencia_nota_actual);


}

/**
 * @brief	Pone a 0 el FLAG_NOTA_TIMEOUT. Actualiza la nota cargando su frecuencia y duraci�n.
 			Nos saca por pantalla la nota que vamos a reproducir o si es la �ltima, el mensaje de que todas se han reproducido.
 * @param	this        Puntero al aut�mata que usa la funci�n.
 			
 * @return	
 * @note	Funci�n que se ejecuta cuando CompruebaNotaTimeout devuelve 1 (por el diagrama de estados).
 */

void ActualizaPlayer (fsm_t* this ){
			TipoPlayer *p_player;
			p_player = (TipoPlayer*)(this->user_data);


			piLock (FLAGS_KEY);
			flags &= ~FLAG_NOTA_TIMEOUT; 
			piUnlock (FLAGS_KEY);

			p_player ->posicion_nota_actual ++;

			piLock (STD_IO_BUFFER_KEY);
			if (p_player->posicion_nota_actual < p_player->melodia->num_notas  ){ 
				printf("\n[PLAYER][ActualizaPlayer][NOTA %d de %d][FREC %d][DURA %d]\n",p_player->posicion_nota_actual+1, p_player->melodia->num_notas, p_player->frecuencia_nota_actual,p_player->duracion_nota_actual);
				fflush(stdout);

			}else{
				printf("\n[PLAYER][ActualizaPlayer][Reproducidas todas las notas]\n");
				fflush(stdout);
				piLock (FLAGS_KEY);
				flags |= FLAG_PLAYER_END;
				flags |= FLAG_SYSTEM_END;
				piUnlock (FLAGS_KEY);
				softToneWrite(18,0);
			}
			p_player->frecuencia_nota_actual = p_player->melodia->frecuencias[p_player->posicion_nota_actual];
			p_player->duracion_nota_actual = p_player->melodia->duraciones[p_player->posicion_nota_actual];

			piUnlock (STD_IO_BUFFER_KEY);



}

/**
 * @brief	Pone a 0 el FLAG_PLAYER_STOP. Para la melod�a y nos lo notifica con un mensaje por pantalla.
 * @param	this        Puntero al aut�mata que usa la funci�n.
 			
 * @return	
 * @note	Funci�n que se ejecuta cuando CompruebaPlayerStop devuelve 1 (por el diagrama de estados).
 */
void StopPlayer (fsm_t* this){
				
				softToneWrite(18,0);
				piLock (FLAGS_KEY);
				flags &= ~FLAG_PLAYER_STOP;
				piUnlock (FLAGS_KEY);

				piLock (STD_IO_BUFFER_KEY);
				printf("\n[PLAYER][StopPlayer]\n");
				fflush(stdout);
				piUnlock (STD_IO_BUFFER_KEY);



}

/**
 * @brief	Pone a 0 el FLAG_PLAYER_END. 
 			Nos saca por pantalla que la melod�a se ha acabado.
 * @param	this        Puntero al aut�mata que usa la funci�n.
 			
 * @return	
 * @note	Funci�n que se ejecuta cuando CompruebaFinalMelod�a devuelve 1 (por el diagrama de estados).
 */
void FinalMelodia(fsm_t* this){  

				piLock (FLAGS_KEY);
				flags &= ~FLAG_PLAYER_END;
				piUnlock (FLAGS_KEY);

				piLock (STD_IO_BUFFER_KEY);
				printf("\n[PLAYER][Final De La Melodia]\n");
				fflush(stdout);
				piUnlock (STD_IO_BUFFER_KEY);



}


/**
 * @brief	Pone a 0 el FLAG_PLAYER_END.  Reproduce la nota actual.
 			Timer?
 * @param	this        Puntero al aut�mata que usa la funci�n.
 			
 * @return	
 * @note	Funci�n que se ejecuta cuando CompruebaNuevaNota devuelve 1 (por el diagrama de estados).
 */
void ComienzaNuevaNota(fsm_t* this){
				TipoPlayer *p_player;
				p_player = (TipoPlayer*)(this->user_data);
				softToneWrite(18,p_player->frecuencia_nota_actual);

				tmr_startms(timer_tmr, p_player->duracion_nota_actual); 

				piLock (FLAGS_KEY);
				flags &= ~FLAG_PLAYER_END;
				piUnlock (FLAGS_KEY);




}


/*--------------------------------------------------------------------------------------
 *                        FUNCIONES DE SALIDA MAQUINA DE ESTADOS 2
 ---------------------------------------------------------------------------------------
*/

/**
 * @brief Nos saca un mensaje por pantalla que nos informa de que podemos ya introducir una tarjeta. Esto
 * se podr� cuando se pulse el bot�n de start: [s]	
 * @param	this        Puntero al aut�mata que usa la funci�n. (Aut�mata 2)
 *			
 * @return	
 * @note	Funci�n que se ejecuta cuando CompruebaComienzo devuelve 1 (por el diagrama de estados).
 */

void ComienzaSistema (fsm_t* this){
	piLock (STD_IO_BUFFER_KEY);
	printf("\nEl sistema esta preparado para leer tarjetas\n");
	piUnlock (STD_IO_BUFFER_KEY);


}

/**
 * @brief Es la funci�n que se dispara mientras no se detecte que se ha metido una tarjeta en el 
 * optoacoplador, es decir, que se ejecuta mientras FLAG_CARD_IN siga a 0. 
 * @param	this    Puntero al aut�mata que usa la funci�n. (Aut�mata 2)
 *			
 * @return	
 * @note	Funci�n que se ejecuta cuando TarjetaNoDisponible devuelve 1 (por el diagrama de estados).
 */
void EsperoTarjeta (fsm_t* this){
}

/**
 * @brief Esta funci�n salta cuando la tarjeta detectada no coincide con ninguna de las tarjetas almacenadas
 * en los arrays de arriba. Nos informa por pantalla de que la tarjeta no es v�lida.	
 * @param	this   Puntero al aut�mata que usa la funci�n. (Aut�mata 2)
 *			
 * @return	
 * @note	Funci�n que se ejecuta cuando TarjetaNoValida devuelve 1 (por el diagrama de estados). El valor de
 * FLAG_VALID_CARD depende del resultado del bucle de LeerTarjeta.
 */
void DescartaTarjeta (fsm_t* this){
	piLock (STD_IO_BUFFER_KEY);
	printf("\nTarjeta no v�lida\n");
		piUnlock (STD_IO_BUFFER_KEY);

		piLock (FLAGS_KEY);
		flags &= ~FLAG_CARD_IN;
		piUnlock (FLAGS_KEY);



}

/**
 * @brief  Recorrer� los arrays definidos al principio, para comprobar si el UID de la tarjeta le�da (par�metro que devuelve la
 * la funci�n read_id()) coincide o no con alguno de los UIDs almacenados. Si coincide activar� el FLAG_VALID_CARD para que la m�quina
 * de estados ejecute ComienzaReproduccion y empiece la m�sica. Si no, dejar� el flag a 0 para que siga esperando otra tarjeta.
 * @param	this        Puntero al aut�mata que usa la funci�n. (Aut�mata 2)
 *			
 * @return	
 * @note	Funci�n que se ejecuta cuando CompruebaComienzo devuelve 1 (por el diagrama de estados).
 */

void LeerTarjeta (fsm_t* this){
	
	TipoSistema *p_sistema;
	p_sistema=(TipoSistema*)(this->user_data);
	UIDTarjetaLeida = read_id();
  
	int i;
	

	for(i=0; i<4; i++){
		if(strcmp(UIDTarjetaLeida,uid[i]) == 0){

			p_sistema->uid_tarjeta_actual_string = UIDTarjetaLeida;
			p_sistema->pos_tarjeta_actual=i;

			piLock (STD_IO_BUFFER_KEY);
			printf("\nTarjeta valida \n");
			piUnlock (STD_IO_BUFFER_KEY);

			piLock (FLAGS_KEY);
			flags |= FLAG_VALID_CARD;
			piUnlock (FLAGS_KEY);
		}
	}





}

/**
 * @brief Es la funci�n encargada de llamar a la m�quina de estados que se encarga de reproducir la melod�a. Llama a
 * InicializaMelodia con la melod�a correspondiente y pone FLAG_PLAYER_START a 1.
 * 
 * @param	this     Puntero al aut�mata que usa la funci�n. (Aut�mata 2)
 *			
 * @return	
 * @note	Funci�n que se ejecuta cuando TarjetaValida devuelve 1 (por el diagrama de estados).
 */

void ComienzaReproduccion (fsm_t* this){
			TipoSistema *p_sistema;
			p_sistema=(TipoSistema*)(this->user_data);
			int posicion = p_sistema->pos_tarjeta_actual;

			InicializaMelodia(p_sistema->player.melodia,
			nombres[posicion],frecuencias[posicion],tiempos[posicion],numNotas[posicion]);

				piLock (FLAGS_KEY);
				flags |= FLAG_PLAYER_START;
				piUnlock (FLAGS_KEY);
}
/**
 * @brief Esta funci�n comprueba si la tarjeta sigue introducida. Si el lector lee "0000000" quiere decir que la tarjeta
 * ya no est� dentro, y entonces pone FLAG_CARD_IN a 0, lo que parar� la melod�a (ya que salta cancela reproducci�n). 
 * Esto ya lo hace la funci�n que responde a la interrrupci�n del optoacoplador cuando deja de detectar la tarjea, pero as�
 * nos aseguramos.
 * 
 * @param	this     Puntero al aut�mata que usa la funci�n. (Aut�mata 2)
 *			
 * @return	
 * @note	Funci�n que se ejecuta cuando TarjetaNoDisponible devuelve 1 (por el diagrama de estados).
 */

void ComprueboTarjeta (fsm_t* this){

	if(strcmp(UIDTarjetaLeida,"00000000") == 0){

				piLock (FLAGS_KEY);
				flags &= ~FLAG_CARD_IN;
				piUnlock (FLAGS_KEY);
			}



}
/**
 * @brief Esta funci�n salta cuando la melod�a ha acabado (cuando el FLAG_SYSTEM_END es puesto a uno por ActualizaPlayer). 
 * Pone los flags correspondientes a 0 para que el sistema pueda volver a leer otra tarjeta.
 * 
 * @param	this     Puntero al aut�mata que usa la funci�n. (Aut�mata 2)
 *			
 * @return	
 * @note	Funci�n que se ejecuta cuando CompruebaFinalReproduccion devuelve 1 (por el diagrama de estados).
 */

void FinalizaReproduccion (fsm_t* this){

				piLock (FLAGS_KEY);
				flags &= ~FLAG_SYSTEM_END;
				flags &= ~FLAG_CARD_IN;
				flags &= ~FLAG_VALID_CARD;
				piUnlock (FLAGS_KEY);


}
/**
 * @brief	Funci�n que se ejecuta cuando se saca la tarjeta, activa y desactiva los flags correspondientes para que pare 
 * la melod�a y se pueda leer otra tarjeta de nuevo. Nos informa por pantalla que la tarjeta ha sido extra�da.
 * @param	this        Puntero al aut�mata que usa la funci�n.
 			
 * @return	
 * @note	Funci�n que se ejecuta cuando TarjetaNoDisponible devuelve 1 (por el diagrama de estados).
 */
void CancelaReproduccion (fsm_t* this){

				piLock (STD_IO_BUFFER_KEY);
				printf("\nTarjeta extra�da \n");
				piUnlock (STD_IO_BUFFER_KEY);

				piLock (FLAGS_KEY);
				flags |= FLAG_PLAYER_STOP;
				flags &= ~FLAG_SYSTEM_END;
				flags &= ~FLAG_CARD_IN;
				flags &= ~FLAG_VALID_CARD;
				piUnlock (FLAGS_KEY);


}


//---------------------------------------------------------------------
// FUNCIONES DE ENTRADA O DE TRANSICION DE LA MAQUINA DE ESTADOS
//---------------------------------------------------------------------

/**
 * @brief	Su funci�n es comprobar el valor de FLAG_PLAYER_START
 			
 * @param	this        Puntero al aut�mata que usa la funci�n.
 			
 * @return	Devuelve el resultado del and de flags y FLAG_PLAYER_START.
 * @note	Funci�n de entrada. Dependiendo de result saltar� o no la funci�n de salida.
 */
int CompruebaPlayerStart (fsm_t* this){
		int result;

		piLock (FLAGS_KEY);
		result = (flags & FLAG_PLAYER_START); 
		piUnlock (FLAGS_KEY);

		return result;
}

/**
 * @brief	Su funci�n es comprobar el valor de FLAG_PLAYER_END
 			
 * @param	this        Puntero al aut�mata que usa la funci�n.
 			
 * @return	Devuelve el resultado del and de flags y FLAG_PLAYER_END.
 * @note	Funci�n de entrada. Dependiendo de result saltar� o no la funci�n de salida.
 */
int CompruebaFinalMelodia (fsm_t* this){
			int result =0;

			piLock (FLAGS_KEY);
			result = (flags & FLAG_PLAYER_END);
			piUnlock (FLAGS_KEY);

			return result;

}

/**
 * @brief	Su funci�n es comprobar el valor de FLAG_NOTA_TIMEOUT.
 			
 * @param	this        Puntero al aut�mata que usa la funci�n.
 			
 * @return	Devuelve el resultado del and de flags y FLAG_NOTA_TIMEOUT.
 * @note	Funci�n de entrada. Dependiendo de result saltar� o no la funci�n de salida.
 */
int CompruebaNotaTimeout (fsm_t* this){
			int result;

			piLock (FLAGS_KEY);
			result = (flags & FLAG_NOTA_TIMEOUT);
			piUnlock (FLAGS_KEY);

			return result;

}

/**
 * @brief	Su funci�n es comprobar el valor de FLAG_PLAYER_END.
 			
 * @param	this        Puntero al aut�mata que usa la funci�n.
 			
 * @return	Devuelve el resultado del and de flags y FLAG_PLAYER_END negado. (ya que hay dos posibles salidas del estado WAIT_END).
 * @note	Funci�n de entrada. Dependiendo de result saltar� o no la funci�n de salida.
 */
int CompruebaNuevaNota (fsm_t* this){
			int result;

			piLock (FLAGS_KEY);
			result = ~(flags & FLAG_PLAYER_END);
			piUnlock (FLAGS_KEY);

			return result;

}

/**
 * @brief	Su funci�n es comprobar el valor de FLAG_PLAYER_STOP
 			
 * @param	this        Puntero al aut�mata que usa la funci�n.
 			
 * @return	Devuelve el resultado del and de flags y FLAG_PLAYER_STOP.
 * @note	Funci�n de entrada. Dependiendo de result saltar� o no la funci�n de salida.
 */

int CompruebaPlayerStop (fsm_t* this){
			int result;

			piLock (FLAGS_KEY);
			result = (flags & FLAG_PLAYER_STOP);
			piUnlock (FLAGS_KEY);

			return result;
}

/**
 * @brief	Su funci�n es comprobar el valor de FLAG_SYSTEM_START
 			
 * @param	this        Puntero al aut�mata que usa la funci�n.
 			
 * @return	Devuelve el resultado del and de flags y FLAG_SYSTEM_START.
 * @note	Funci�n de entrada. Dependiendo de result saltar� o no la funci�n de salida.
 */

int CompruebaComienzo (fsm_t* this){
			int result;

			piLock (FLAGS_KEY);
			result = (flags & FLAG_SYSTEM_START);
			piUnlock (FLAGS_KEY);

			return result;
}

/**
 * @brief	Su funci�n es comprobar el valor de FLAG_CARD_IN
 			
 * @param	this        Puntero al aut�mata que usa la funci�n.
 			
 * @return	Devuelve el resultado del and de flags y FLAG_CARD_IN todo ello negado. (Para 
 * que salte cuando FLAG_CARD_IN sea 0).
 * @note	Funci�n de entrada. Dependiendo de result saltar� o no la funci�n de salida. 
 */

int TarjetaNoDisponible (fsm_t* this){
			int result;

			piLock (FLAGS_KEY);
			result = ~(flags & FLAG_CARD_IN);
			piUnlock (FLAGS_KEY);

			return result;
}

/**
 * @brief	Su funci�n es comprobar el valor de FLAG_CARD_IN
 			
 * @param	this        Puntero al aut�mata que usa la funci�n.
 			
 * @return	Devuelve el resultado del and de flags y FLAG_CARD_IN.
 * @note	Funci�n de entrada. Dependiendo de result saltar� o no la funci�n de salida. 
 */

int  TarjetaDisponible (fsm_t* this){
			int result;

			piLock (FLAGS_KEY);
			result = (flags & FLAG_CARD_IN);
			piUnlock (FLAGS_KEY);

			return result;
}

/**
 * @brief	Su funci�n es comprobar el valor de FLAG_VALID_CARD
 			
 * @param	this        Puntero al aut�mata que usa la funci�n.
 			
 * @return	Devuelve el resultado del and de flags y FLAG_VALID_CARD todo ello negado. (Para 
 * que salte cuando FLAG_VALID_CARD sea 0).
 * @note	Funci�n de entrada. Dependiendo de result saltar� o no la funci�n de salida. 
 */

int TarjetaNoValida (fsm_t* this){
			int result;

			piLock (FLAGS_KEY);
			result = ~(flags & FLAG_VALID_CARD);
			piUnlock (FLAGS_KEY);

			return result;
}

/**
 * @brief	Su funci�n es comprobar el valor de FLAG_VALID_CARD
 			
 * @param	this        Puntero al aut�mata que usa la funci�n.
 			
 * @return	Devuelve el resultado del and de flags y FLAG_VALID_CARD.
 * @note	Funci�n de entrada. Dependiendo de result saltar� o no la funci�n de salida. 
 */

int TarjetaValida (fsm_t* this){
			int result;

			piLock (FLAGS_KEY);
			result = (flags & FLAG_VALID_CARD);
			piUnlock (FLAGS_KEY);

			return result;
}

/**
 * @brief	Su funci�n es comprobar el valor de FLAG_SYSTEM_END
 			
 * @param	this        Puntero al aut�mata que usa la funci�n.
 			
 * @return	Devuelve el resultado del and de flags y FLAG_SYSTEM_END
 * @note	Funci�n de entrada. Dependiendo de result saltar� o no la funci�n de salida. 
 */

int CompruebaFinalReproduccion (fsm_t* this){
			int result;

			piLock (FLAGS_KEY);
			result = (flags & FLAG_SYSTEM_END);
			piUnlock (FLAGS_KEY);

			return result;
}




//-------------------------------------------------------------------------------------
// 							FUNCIONES DE INICIALIZACION
//-------------------------------------------------------------------------------------



/*
La detecci�n de las teclas pulsadas lo hacemos con un proceso paralelo al del programa principal. 
Este hilo (PI_THREAD) comparte las variables globales con el programa principal.
Para comunicar los dos procesos que van a actuar se usar� la varible flags creada al principio del
programa (que como ya hemos dicho es com�n a ambos procesos).
La tecla "s" hace que se puedan a empezar a leer tarjetas (Pone a 1 el FLAG_SYSTEM_START). La tecla "q"
nos permite salir del programa. Otras teclas no son admitidas y nos informa de ello.
 */

PI_THREAD (thread_explora_teclado) {
	char teclaPulsada;

	while(1) {
		delay(10); // Pausa el programa durante 10 ms

		if(kbhit()) {
			teclaPulsada = kbread(); 
			piLock (STD_IO_BUFFER_KEY);
			printf("\n[KBHIT][%c]\n", teclaPulsada);
			piUnlock (STD_IO_BUFFER_KEY);

			switch(teclaPulsada) {
				case 's':
					piLock (FLAGS_KEY);
					flags |= FLAG_SYSTEM_START;
					piUnlock (FLAGS_KEY);
					break;

				case 'q':
					exit(0);
					break;

				default:

					piLock (STD_IO_BUFFER_KEY);
					printf("INVALID KEY!!! \n"); 
					fflush(stdout);
					piUnlock (STD_IO_BUFFER_KEY);


					break;
			}
		}

		
	}
}

/*
Funci�n que se ejecuta cuando ocurre la interrupci�n configurada en el pin 1 (GPIO 5). Si se detecta una tarjeta (flanco 
de bajada) entonces se pone FLAG_CARD_IN a 1. Si por el contrario se detecta un flanco de subida se pondr� el FLAG a 0, 
ya que significa que la tarjeta se ha sacado.
Tambi�n ponemos las sentencias necesarias para evitar los posibles rebotes.
 */

void detecta_tarjeta(){
	if(millis () < debounceTime){ 
		debounceTime = millis () + 500 ;
	}

	if (digitalRead(5) == LOW) {
		piLock (FLAGS_KEY);

		flags |= FLAG_CARD_IN;
		piUnlock (FLAGS_KEY);

	} else {
		piLock (FLAGS_KEY);

		flags &= ~ FLAG_CARD_IN;
		piUnlock (FLAGS_KEY);
	}

	debounceTime = millis() + 500;
}


 /* @brief  	Funci�n que configura el sistema y que nos saca por pantalla los problemas que puedan haber ocurrido.
  * Inicializa la librer�a wiringPi, lanza el thread para exploracion
  * del teclado del PC  bloqueando y liberando el mutex de pantalla para poder mostrar mensajes con seguridad. Tambi�n 
  * configura la interrupci�n externa asociada al pin GPIO 5 (La correspondiente al optoacoplador) e inicializa el lector 
  * de tarjetas.
 			
 * @param	
 			
 * @return	Devuelve 1 si todo ha ido bien. Si no devuelve -1.
 * @note	
 */ 

int systemSetup (void) {
	int x = 0;

		piLock (STD_IO_BUFFER_KEY);

		
		if (wiringPiSetupGpio () < 0) {
			printf ("Unable to setup wiringPi\n");
			piUnlock (STD_IO_BUFFER_KEY);
			return -1;
	    }

		
		x = piThreadCreate (thread_explora_teclado);

		if (x != 0) {
			printf ("it didn't start!!!\n");
			piUnlock (STD_IO_BUFFER_KEY);
			return -1;
	    }


		piUnlock (STD_IO_BUFFER_KEY);

		pinMode(5, INPUT);
		wiringPiISR(5, INT_EDGE_BOTH, &detecta_tarjeta); 
		softToneCreate(18);

		initialize_rfid();
		
		return 1;
	}

/**
 * @brief  	Funci�n que inicializa el aut�mata. Pone todos los flags a 0.

 			
 * @param	player_fsm    Puntero al aut�mata que usa la funci�n.
 			
 * @return	
 * @note	
 */ 

void fsm_setup (fsm_t* player_fsm){
	piLock (FLAGS_KEY);
	flags = 0;
	piUnlock (FLAGS_KEY);
}

void delay_until (unsigned int next){
	unsigned int now = millis();
	if(next>now){
		delay (next - now);
	}
}
/* Programa principal. En el programa principal. Creamos un TipoSistema, un TipoMelodia y un TipoPlayer. 
	Creamos el array reproductor[] y el tarjeta[] que corresponden con las tablas de transiciones de las 
	m�quinas de estados.  
	En la siguiente l�nea vamos a pasar a fsm_new los datos adecuados para generar las m�quinas de estados:
	estado inicial, nombre del array con las transiciones y datos adicionales. 
	Tambi�n definimos el objeto timer y le dices a qu� funci�n tiene que ir cuando acaba la cuenta.
	Llamamos a systemSetUp() para configurar todo y por �ltimo escribimos el bucle while infinito que ejecuta 
	el aut�mata llamando a la funci�n fsm_fire que ejecuta las transiciones de los aut�matas.
	

*/

int main () {

	fflush(stdout);
	TipoSistema sistema;
	TipoMelodia melodia;
	TipoPlayer p_player; 
	unsigned int next;



// M�quinas de estados: lista de transiciones
// {EstadoOrigen,Funci�nDeEntrada,EstadoDestino,Funci�nDeSalida}
	fsm_trans_t reproductor[] ={
				{WAIT_START, CompruebaPlayerStart,WAIT_NEXT,InicializaPlayer},
				{WAIT_NEXT, CompruebaNotaTimeout,WAIT_END,ActualizaPlayer},
				{WAIT_END, CompruebaFinalMelodia,WAIT_START,FinalMelodia},
				{WAIT_END, CompruebaNuevaNota,WAIT_NEXT,ComienzaNuevaNota},
				{WAIT_NEXT, CompruebaPlayerStop,WAIT_START,StopPlayer},
				{-1,NULL,-1,NULL},
		};

		fsm_trans_t tarjeta[] ={
				{WAIT_START, CompruebaComienzo, WAIT_CARD, ComienzaSistema},
				{WAIT_CARD, TarjetaDisponible, WAIT_CHECK, LeerTarjeta},
				{WAIT_CARD, TarjetaNoDisponible, WAIT_CARD, EsperoTarjeta},
				{WAIT_CHECK, TarjetaValida, WAIT_PLAY, ComienzaReproduccion},
				{WAIT_CHECK, TarjetaNoValida, WAIT_CARD, DescartaTarjeta},
				{WAIT_PLAY, TarjetaDisponible, WAIT_PLAY, ComprueboTarjeta},
				{WAIT_PLAY, CompruebaFinalReproduccion, WAIT_START, FinalizaReproduccion},
				{WAIT_PLAY, TarjetaNoDisponible, WAIT_START, CancelaReproduccion},
				{-1,NULL,-1,NULL},
		};

fsm_t* player_fsm = fsm_new (WAIT_START, reproductor, &(sistema.player));
fsm_t* tarjeta_fsm = fsm_new (WAIT_START, tarjeta, &(sistema));


p_player.melodia = &melodia;
sistema.player = p_player;


timer_tmr = tmr_new(timer_isr); 

fsm_setup (player_fsm);
fsm_setup (tarjeta_fsm);

systemSetup();


next = millis();

while (1) {
fsm_fire (player_fsm);
fsm_fire (tarjeta_fsm);
next += CLK_MS;
delay_until (next);
}
fsm_destroy (player_fsm);
fsm_destroy (tarjeta_fsm);
tmr_destroy (timer_tmr); 
}


