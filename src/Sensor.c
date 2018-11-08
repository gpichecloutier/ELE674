/*
 * Sensor.c
 *
 *  Created on: 12 sept. 2013
 *      Author: Gabriel Piché Cloutier, Francis Jeanneau
 *
 */

#include "Sensor.h"

#define ABS(x) (((x) < 0.0) ? -(x) : (x))


#define MAX_TOT_SAMPLE 1000

extern SensorStruct	SensorTab[NUM_SENSOR];

pthread_barrier_t   SensorStartBarrier;
pthread_barrier_t   LogStartBarrier;
pthread_mutex_t 	  Log_Mutex;

uint8_t  SensorsActivated 	= 0;
uint8_t  LogActivated  	  	= 0;
uint8_t  numLogOutput 	  	= 0;


void *SensorTask ( void *ptr ) {
/* A faire! */
/* Tache qui sera instancié pour chaque sensor. Elle s'occupe d'aller */
/* chercher les donnees du sensor.                                    */

	int i, j;
	uint32_t last_timestamp;
	uint32_t new_timestamp;

	SensorStruct *sensor_struct;
	sensor_struct = (SensorStruct*)ptr;
	SensorData sensor_data_temp;
	SensorRawData sensor_raw_data_temp;


	pthread_barrier_wait(&(SensorStartBarrier));

	// Pour printer
//	j = 0;

	while (SensorsActivated) {

//		pthread_mutex_lock(&(sensor_data->DataSampleMutex));
//		pthread_spin_lock(&(sensor_data->DataLock));

//		last_timestamp = sensor_raw_data_temp.timestamp_n + (sensor_raw_data_temp.timestamp_s * (10^9));
		last_timestamp = sensor_struct->RawData[sensor_struct->DataIdx]->timestamp_n + (sensor_struct->RawData[sensor_struct->DataIdx]->timestamp_s * (10^9));

		// Le read est bloquant alors pas de spin lock
		if ((read(sensor_struct->File, &sensor_raw_data_temp, sizeof(sensor_raw_data_temp))) == sizeof(sensor_raw_data_temp)){
			// Les données ont été lues et placées dans "RawData"


			// Conversion du RawData en Data
			// EST-CE QU'ON LOCK POUR LIRE LE TYPE ET CENTERVAL ET AUTRES???
			switch (sensor_struct->type) {
			case ACCELEROMETRE :	for (i = 0; i < 3; i++)
										sensor_data_temp.Data[i] = ((sensor_raw_data_temp.data[i] - sensor_struct->Param->centerVal) * sensor_struct->Param->Conversion);
									break;

			case GYROSCOPE :		for (i = 0; i < 3; i++)
										sensor_data_temp.Data[i] = ((sensor_raw_data_temp.data[i] - sensor_struct->Param->centerVal) * sensor_struct->Param->Conversion);
									break;

			case SONAR :			sensor_data_temp.Data[0] = ((sensor_raw_data_temp.data[0] - sensor_struct->Param->centerVal) * sensor_struct->Param->Conversion);
									break;

			case BAROMETRE :		sensor_data_temp.Data[0] = ((sensor_raw_data_temp.data[0] - sensor_struct->Param->centerVal) * sensor_struct->Param->Conversion);
									break;

			case MAGNETOMETRE :		for (i = 0; i < 3; i++)
										sensor_data_temp.Data[i] = (((double)sensor_raw_data_temp.data[i] - sensor_struct->Param->centerVal) * sensor_struct->Param->Conversion);
									break;
			}

			// Calcul du TimeDelay
			new_timestamp = sensor_raw_data_temp.timestamp_n + (sensor_raw_data_temp.timestamp_s * (10^9));
			sensor_data_temp.TimeDelay = new_timestamp - last_timestamp;

//			if (j > 200) {
//				printf("Valeur du sensor %s en x : %lf, en y : %lf, en z : %lf\n", sensor_data->Name, sensor_data->Data->Data[0], sensor_data->Data->Data[1], sensor_data->Data->Data[2]);
//				j = 0;
//			}
//
//			j++;

			// On "lock" pour assigner le data au data_temp...
			pthread_mutex_lock(&(sensor_struct->DataSampleMutex));
			pthread_spin_lock(&(sensor_struct->DataLock));

			// 1. Incrémenter l'index
			sensor_struct->DataIdx = (sensor_struct->DataIdx + 1) % DATABUFSIZE;

			// 2. Copier les données
			memcpy((void *) &(sensor_data_temp),     (void *) (sensor_struct->Data[sensor_struct->DataIdx]),    sizeof(sensor_data_temp));
			memcpy((void *) &(sensor_raw_data_temp), (void *) (sensor_struct->RawData[sensor_struct->DataIdx]), sizeof(sensor_raw_data_temp));

			pthread_cond_broadcast(&(sensor_struct->DataNewSampleCondVar));
			pthread_spin_unlock(&(sensor_struct->DataLock));
			pthread_mutex_unlock(&(sensor_struct->DataSampleMutex));




		} else {
			//La structure n'a pas été copiée en entier


		}

	}
	pthread_exit(0); /* exit thread */
}


int SensorsInit (SensorStruct SensorTab[NUM_SENSOR]) {
/* A faire! */
/* Ici, vous devriez faire l'initialisation de chacun des capteurs.  */ 
/* C'est-à-dire de faire les initialisations requises, telles que    */
/* ouvrir les fichiers des capteurs, et de créer les Tâches qui vont */
/* s'occuper de réceptionner les échantillons des capteurs.          */

	int retval;
	pthread_attr_t      attr;
	struct sched_param	param;
	int i;

	pthread_attr_init(&attr);
	pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
	pthread_attr_setscope(&attr, PTHREAD_SCOPE_PROCESS);
	pthread_attr_setschedpolicy(&attr, POLICY);
	param.sched_priority = sched_get_priority_min(POLICY);
	pthread_attr_setstacksize(&attr, THREADSTACK);
	pthread_attr_setschedparam(&attr, &param);

	printf("Initialisation des Sensors...\n");

	// Initiatisation de la barrière
	pthread_barrier_init(&SensorStartBarrier, NULL, NUM_SENSOR + 1);

	for (i = ACCELEROMETRE; i <= MAGNETOMETRE; i++) {
		// Ouverture des pilotes des capteurs
		if((retval = SensorTab[i].File  = open(SensorTab[i].DevName, O_RDONLY)) < 0) {
			printf("%s : Impossible d'ouvrir le pilote de %s => retval = %d\n", __FUNCTION__, SensorTab[i].Name, retval);
			return -1;
		}

		// Initialisation des tâches
		if((retval = pthread_create(&(SensorTab[i].SensorThread), &attr, SensorTask, (void *) &SensorTab[i])) != 0) {
			printf("%s : Impossible de créer la Tâche de %s => retval = %d\n", __FUNCTION__, SensorTab[i].Name, retval);
			return -1;
		}

		// Initialisation des spinlock
		if((retval = pthread_spin_init(&(SensorTab[i].DataLock), PTHREAD_PROCESS_PRIVATE)) != 0) {
			printf("%s : Impossible de créer le spinlock de %s => retval = %d\n", __FUNCTION__, SensorTab[i].Name, retval);
			return -1;
		}

		// Initialisation des mutex
		if((retval = pthread_mutex_init(&(SensorTab[i].DataSampleMutex), NULL)) != 0) {
			printf("%s : Impossible de créer le mutex de %s => retval = %d\n", __FUNCTION__, SensorTab[i].Name, retval);
			return -1;
		}

		// Initialisation des variables de condition
		if((retval = pthread_cond_init(&(SensorTab[i].DataNewSampleCondVar), NULL)) != 0) {
			printf("%s : Impossible de créer la variable de condition de %s => retval = %d\n", __FUNCTION__, SensorTab[i].Name, retval);
			return -1;
		}

		// Initialisation des timestamps
		SensorTab[i].RawData->timestamp_n = 0;
		SensorTab[i].RawData->timestamp_s = 0;
	}

	pthread_attr_destroy(&attr);

	printf("Tout a été crééééyeees...\n");

	return 0;
};


int SensorsStart (void) {
/* A faire! */
/* Ici, vous devriez démarrer l'acquisition sur les capteurs.        */ 
/* Les capteurs ainsi que tout le reste du système devrait être      */
/* prêt à faire leur travail et il ne reste plus qu'à tout démarrer. */
	SensorsActivated = 1;

	pthread_barrier_wait(&(SensorStartBarrier));
	pthread_barrier_destroy(&SensorStartBarrier);
	printf("%s Sensors démarrés\n", __FUNCTION__);

	return 0;
}


int SensorsStop (SensorStruct SensorTab[NUM_SENSOR]) {
/* A faire! */
/* Ici, vous devriez défaire ce que vous avez fait comme travail dans */
/* SensorsInit() (toujours verifier les retours de chaque call)...    */ 
	int16_t	i;

	SensorsActivated = 0;
	for (i = ACCELEROMETRE; i <= MAGNETOMETRE; i++) {
		pthread_cancel(SensorTab[i].SensorThread);
		pthread_spin_destroy(&(SensorTab[i].DataSampleMutex));
		pthread_mutex_destroy(&(SensorTab[i].DataSampleMutex));
		pthread_cond_destroy(&(SensorTab[i].DataNewSampleCondVar));
	}

	return 0;
}



/* Le code ci-dessous est un CADEAU !!!	*/
/* Ce code permet d'afficher dans la console les valeurs reçues des capteurs.               */
/* Évidemment, celà suppose que les acquisitions sur les capteurs fonctionnent correctement. */
/* Donc, dans un premier temps, ce code peut vous servir d'exemple ou de source d'idées.     */
/* Et dans un deuxième temps, peut vous servir pour valider ou vérifier vos acquisitions.    */
/*                                                                                           */
/* NOTE : Ce code suppose que les échantillons des capteurs sont placés dans un tampon       */
/*        circulaire de taille DATABUFSIZE, tant pour les données brutes (RawData) que       */
/*        les données converties (NavData) (voir ci-dessous)                                 */
void *SensorLogTask ( void *ptr ) {
	SensorStruct	*Sensor    = (SensorStruct *) ptr;
	uint16_t		*Idx       = &(Sensor->DataIdx);
	uint16_t		LocalIdx   = DATABUFSIZE;
	SensorData	 	*NavData   = NULL;
	SensorRawData	*RawData   = NULL;
	SensorRawData   tpRaw;
	SensorData 	    tpNav;
	double			norm;

	printf("%s : Log de %s prêt à démarrer\n", __FUNCTION__, Sensor->Name);
	pthread_barrier_wait(&(LogStartBarrier));

	while (LogActivated) {
		pthread_mutex_lock(&(Sensor->DataSampleMutex));
		while (LocalIdx == *Idx)
			pthread_cond_wait(&(Sensor->DataNewSampleCondVar), &(Sensor->DataSampleMutex));
	    pthread_mutex_unlock(&(Sensor->DataSampleMutex));

	   	pthread_spin_lock(&(Sensor->DataLock));
    	NavData   = &(Sensor->Data[LocalIdx]);
    	RawData   = &(Sensor->RawData[LocalIdx]);
		memcpy((void *) &tpRaw, (void *) RawData, sizeof(SensorRawData));
		memcpy((void *) &tpNav, (void *) NavData, sizeof(SensorData));
	   	pthread_spin_unlock(&(Sensor->DataLock));

	   	pthread_mutex_lock(&Log_Mutex);
		if (numLogOutput == 0)
			printf("Sensor  :     TimeStamp      SampleDelay  Status  SampleNum   Raw Sample Data  =>        Converted Sample Data               Norme\n");
		else switch (tpRaw.type) {
				case ACCELEROMETRE :	norm = sqrt(tpNav.Data[0]*tpNav.Data[0]+tpNav.Data[1]*tpNav.Data[1]+tpNav.Data[2]*tpNav.Data[2]);
										printf("Accel   : (%5u.%09d)-(0.%09u)  %2d     %8u   %04X  %04X  %04X  =>  %10.5lf  %10.5lf  %10.5lf  =  %10.5lf\n", tpRaw.timestamp_s, tpRaw.timestamp_n, tpNav.TimeDelay, tpRaw.status, tpRaw.ech_num, (uint16_t) tpRaw.data[0], (uint16_t) tpRaw.data[1], (uint16_t) tpRaw.data[2], tpNav.Data[0], tpNav.Data[1], tpNav.Data[2], norm);
										break;
				case GYROSCOPE :		norm = sqrt(tpNav.Data[0]*tpNav.Data[0]+tpNav.Data[1]*tpNav.Data[1]+tpNav.Data[2]*tpNav.Data[2]);
										printf("Gyro    : (%5u.%09d)-(0.%09u)  %2d     %8u   %04X  %04X  %04X  =>  %10.5lf  %10.5lf  %10.5lf  =  %10.5lf\n", tpRaw.timestamp_s, tpRaw.timestamp_n, tpNav.TimeDelay, tpRaw.status, tpRaw.ech_num, (uint16_t) tpRaw.data[0], (uint16_t) tpRaw.data[1], (uint16_t) tpRaw.data[2], tpNav.Data[0], tpNav.Data[1], tpNav.Data[2], norm);
										break;
				case SONAR :			printf("Sonar   : (%5u.%09d)-(0.%09u)  %2d     %8u   %04X              =>  %10.5lf\n", tpRaw.timestamp_s, tpRaw.timestamp_n, tpNav.TimeDelay, tpRaw.status, tpRaw.ech_num, (uint16_t) tpRaw.data[0], tpNav.Data[0]);
										break;
				case BAROMETRE :		printf("Barom   : (%5u.%09d)-(0.%09u)  %2d     %8u   %04X              =>  %10.5lf\n", tpRaw.timestamp_s, tpRaw.timestamp_n, tpNav.TimeDelay, tpRaw.status, tpRaw.ech_num, (uint16_t) tpRaw.data[0], tpNav.Data[0]);
										break;
				case MAGNETOMETRE :		norm = sqrt(tpNav.Data[0]*tpNav.Data[0]+tpNav.Data[1]*tpNav.Data[1]+tpNav.Data[2]*tpNav.Data[2]);
										printf("Magneto : (%5u.%09d)-(0.%09u)  %2d     %8u   %04X  %04X  %04X  =>  %10.5lf  %10.5lf  %10.5lf  =  %10.5lf\n", tpRaw.timestamp_s, tpRaw.timestamp_n, tpNav.TimeDelay, tpRaw.status, tpRaw.ech_num, (uint16_t) tpRaw.data[0], (uint16_t) tpRaw.data[1], (uint16_t) tpRaw.data[2], tpNav.Data[0], tpNav.Data[1], tpNav.Data[2], norm);
										break;
			 }
		LocalIdx = *Idx;
		numLogOutput++;
		if (numLogOutput > 20)
			numLogOutput = 0;
		pthread_mutex_unlock(&Log_Mutex);
	}

	printf("%s : %s Terminé\n", __FUNCTION__, Sensor->Name);

	pthread_exit(0); /* exit thread */
}


int InitSensorLog (SensorStruct *Sensor) {
	pthread_attr_t		attr;
	struct sched_param	param;
	int					retval;

	pthread_attr_init(&attr);
	pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
	pthread_attr_setscope(&attr, PTHREAD_SCOPE_PROCESS);
	pthread_attr_setschedpolicy(&attr, POLICY);
	param.sched_priority = sched_get_priority_min(POLICY);
	pthread_attr_setstacksize(&attr, THREADSTACK);
	pthread_attr_setschedparam(&attr, &param);

	printf("Creating Log thread : %s\n", Sensor->Name);
	if ((retval = pthread_create(&(Sensor->LogThread), &attr, SensorLogTask, (void *) Sensor)) != 0)
		printf("%s : Impossible de créer Tâche Log de %s => retval = %d\n", __FUNCTION__, Sensor->Name, retval);


	pthread_attr_destroy(&attr);

	return 0;
}


int SensorsLogsInit (SensorStruct SensorTab[]) {
	int16_t	  i, numLog = 0;
	int16_t	  retval = 0;

	for (i = 0; i < NUM_SENSOR; i++) {
		if (SensorTab[i].DoLog == 1) {
			if ((retval = InitSensorLog(&SensorTab[i])) < 0) {
				printf("%s : Impossible d'initialiser log de %s => retval = %d\n", __FUNCTION__, SensorTab[i].Name, retval);
				return -1;
			}
			numLog++;
		}
	}
	pthread_barrier_init(&LogStartBarrier, NULL, numLog+1);
	pthread_mutex_init(&Log_Mutex, NULL);

	return 0;
};


int SensorsLogsStart (void) {
	LogActivated = 1;
	pthread_barrier_wait(&(LogStartBarrier));
	pthread_barrier_destroy(&LogStartBarrier);
	printf("%s NavLog démarré\n", __FUNCTION__);

	return 0;
};


int SensorsLogsStop (SensorStruct SensorTab[]) {
	int16_t	i;

	LogActivated = 0;
	for (i = 0; i < NUM_SENSOR; i++) {
		if (SensorTab[i].DoLog == 1) {
			pthread_join(SensorTab[i].LogThread, NULL);
			SensorTab[i].DoLog = 0;
		}
	}
	pthread_mutex_destroy(&Log_Mutex);

	return 0;
};


