/*
 * Motor.c
 *
 *  Created on: 12 sept. 2013
 *  Author: Gabriel Piché Cloutier, Francis Jeanneau
 */


#include "Motor.h"

#define TimeDelay 50000

extern sem_t	MotorTimerSem;
extern int		MotorActivated;

pthread_barrier_t 	MotorStartBarrier;

void SetPWM(MotorStruct *Motor, uint8_t *cmd) {
	uint8_t pwm1 = (uint16_t) Motor->pwm[0];
	uint8_t pwm2 = (uint16_t) Motor->pwm[1];
	uint8_t pwm3 = (uint16_t) Motor->pwm[2];
	uint8_t pwm4 = (uint16_t) Motor->pwm[3];

	cmd[0] = 0x20 | ((pwm1&0x1ff)>>4);
	cmd[1] = ((pwm1&0x1ff)<<4) | ((pwm2&0x1ff)>>5);
	cmd[2] = ((pwm2&0x1ff)<<3) | ((pwm3&0x1ff)>>6);
	cmd[3] = ((pwm3&0x1ff)<<2) | ((pwm4&0x1ff)>>7);
	cmd[4] = ((pwm4&0x1ff)<<1);
}

int gpio_set (int nr, int val)  {
	char cmdline[200];

	if (val < 0)
		sprintf(cmdline, "/usr/sbin/gpio %d -d i", nr);
	else if (val > 0)
		sprintf(cmdline, "/usr/sbin/gpio %d -d ho 1", nr);
	else
		sprintf(cmdline, "/usr/sbin/gpio %d -d ho 0", nr);

	return system(cmdline);
}


int motor_open(void) {
	struct termios config;
	int uart = open(MOTOR_UART, O_RDWR | O_NOCTTY | O_NDELAY);

	if (uart < 0) {
		printf("motor_open : impossible d'ouvrir le uart du moteur\n");
		return uart;
	}

	fcntl(uart, F_SETFL, 0); //read calls are non blocking

	//set port config
	tcgetattr(uart, &config);
	cfsetspeed(&config, B115200);
	config.c_cflag |= (CLOCAL | CREAD); //Enable the receiver and set local mode
	config.c_iflag = 0; //clear input config
	config.c_lflag = 0; //clear local config
	config.c_oflag &= ~OPOST; //clear output config (raw output)
	cfmakeraw(&config);
	tcsetattr(uart, TCSANOW, &config);
	return uart;
}

int motor_cmd(int file, uint8_t cmd, uint8_t *reply, int replylen) {
	// Exemple d'écriture sur le port série.
	int size;

	write(file, &cmd, 1);
	fsync(file);
	usleep(TimeDelay);
	size = read(file, reply, replylen);

	return size;
}

// Reçoit en paramètre l'adresse de la structure Motor_struct. Ça va faire pzzzz, pzzzzz, pzzz, pzzz
int MotorPortInit(MotorStruct *Motor) {
	uint8_t reply[256];
	int		i;

	//open motor port
	Motor->file = motor_open();
	if (Motor->file < 0) {
		printf("motor_open: Impossible d'ouvrir le UART\n");
		return Motor->file;
	}
	//reset IRQ flipflop - this code resets GPIO_ERROR_READ to 0
	gpio_set(GPIO_ERROR_RESET, 0);
	usleep(2*TimeDelay);

	//all select lines inactive
	gpio_set(GPIO_M1, 0);
	gpio_set(GPIO_M2, 0);
	gpio_set(GPIO_M3, 0);
	gpio_set(GPIO_M4, 0);
	usleep(2*TimeDelay);

	//configure motors
	for (i = 0; i < 4; ++i) {
		gpio_set(GPIO_M1 + i, -1);
		usleep(2*TimeDelay);
		motor_cmd(Motor->file, 0xE0, reply, 2);
		motor_cmd(Motor->file, 0x91, reply, 121);
		motor_cmd(Motor->file, 0xA1, reply, 2);
		motor_cmd(Motor->file, i + 1, reply, 1);
		motor_cmd(Motor->file, 0x40, reply, 2);
		gpio_set(GPIO_M1 + i ,0);
		usleep(2*TimeDelay);
	}

	//all select lines active
	gpio_set(GPIO_M1, -1);
	gpio_set(GPIO_M2, -1);
	gpio_set(GPIO_M3, -1);
	gpio_set(GPIO_M4, -1);
	usleep(2*TimeDelay);

	gpio_set(GPIO_ERROR_READ, -1);
	usleep(2*TimeDelay);

	return 0;
}

void *MotorTask ( void *ptr ) {
/* A faire! */
/* Tache qui transmet les nouvelles valeurs de vitesse */
/* à chaque moteur à interval régulier (5 ms).         */

	uint8_t cmd[5];
	MotorStruct *motor_data = (MotorStruct*)ptr;

	pthread_barrier_wait(&(MotorStartBarrier));

	while (MotorActivated) {
		sem_wait(&MotorTimerSem);

		if (MotorActivated == 0)
			break; /* exit while */

		// Bâtit la trame de communication
		pthread_spin_lock(&(motor_data->MotorLock));
		SetPWM(motor_data, cmd);
		pthread_spin_unlock(&(motor_data->MotorLock));

		// Envoie la trame sur le port série
		write(motor_data->file, cmd, 5);
	}
	pthread_exit(NULL); /* exit thread */
}


int MotorInit (MotorStruct *Motor) {
/* A faire! */
/* Ici, vous devriez faire l'initialisation des moteurs.   */
/* C'est-à-dire initialiser le Port des moteurs avec la    */
/* fonction MotorPortInit() et créer la Tâche MotorTask()  */
/* qui va s'occuper des mises à jours des moteurs en cours */
/* d'exécution.                                            */

	pthread_attr_t		attr;
	struct sched_param	param;
	int					minprio, maxprio;
	int i;

	// Créer un semaphore pour cadencer
	sem_init(&MotorTimerSem, 0, 0);
	pthread_barrier_init(&MotorStartBarrier, NULL, 2);
	//	spinlock pour motor_struct
	pthread_spin_init(&(Motor->MotorLock), PTHREAD_PROCESS_SHARED);

	pthread_attr_init(&attr);
	pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
	pthread_attr_setscope(&attr, PTHREAD_SCOPE_SYSTEM);
	minprio = sched_get_priority_min(POLICY);
	maxprio = sched_get_priority_max(POLICY);
	pthread_attr_setschedpolicy(&attr, POLICY);
	param.sched_priority = minprio + (maxprio - minprio) / 2 ;
	pthread_attr_setstacksize(&attr, THREADSTACK);
	pthread_attr_setschedparam(&attr, &param);

	MotorPortInit(Motor);

	for(i=0; i < 4; i++) {
		Motor->pwm[i] = 0;
	}

	// Créer la tâche Moteur avec pthread
	pthread_create(&(Motor->MotorThread), &attr, MotorTask, Motor);

	pthread_attr_destroy(&attr);

	return 0;
}



int MotorStart (void) {
/* A faire! */
/* Ici, vous devriez démarrer la mise à jour des moteurs (MotorTask).    */
/* Tout le système devrait être prêt à faire leur travail et il ne reste */
/* plus qu'à tout démarrer.                                       */

	MotorActivated = 1;
	pthread_barrier_wait(&(MotorStartBarrier));
	pthread_barrier_destroy(&MotorStartBarrier);
	printf("%s Moteur démarré\n", __FUNCTION__);

	return 0;
}



int MotorStop (MotorStruct *Motor) {
/* A faire! */
/* Ici, vous devriez arrêter les moteurs et fermer le Port des moteurs. */
/* Détruire la tâche et les mécanismes de synchronisation */

	int err;

	MotorActivated = 0;
	sem_post(&MotorTimerSem);

	printf("Destruction de la tache moteur\n");
	err = pthread_join(Motor->MotorThread, NULL);

	if (err) {
		printf("pthread_join(Motor->MotorThread) : Erreur\n");
		return err;
	}

	printf("Tache moteur detruite\n");

	pthread_spin_destroy(&(Motor->MotorLock));
	sem_destroy(&MotorTimerSem);

	return err;
}
