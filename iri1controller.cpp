/******************* INCLUDES ******************/
/***********************************************/

/******************** General ******************/
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <sys/time.h>
#include <iostream>

/******************** Simulator ****************/
/******************** Sensors ******************/
#include "epuckproximitysensor.h"
#include "contactsensor.h"
#include "reallightsensor.h"
#include "realbluelightsensor.h"
#include "realredlightsensor.h"
#include "groundsensor.h"
#include "groundmemorysensor.h"
#include "batterysensor.h"
#include "bluebatterysensor.h"
#include "redbatterysensor.h"
#include "encodersensor.h"
#include "compasssensor.h"

/******************** Actuators ****************/
#include "wheelsactuator.h"

/******************** Controller **************/
#include "iri1controller.h"

extern gsl_rng *rng;
extern long int rngSeed;

using namespace std;
/******************************************************************************/
/******************************************************************************/

#define BEHAVIORS	5

#define AVOID_PRIORITY 		0
#define RELOAD_PRIORITY 	1
#define REST_PRIORITY		2
#define TABLE_PRIORITY		3
#define STOP_PRIORITY 		4


#define PROXIMITY_THRESHOLD 0.55
#define BATTERY_THRESHOLD 0.3

#define SPEED 300.0
#define STOP_SPEED 0

/******************************************************************************/
/******************************************************************************/
CIri1Controller::CIri1Controller (const char* pch_name, CEpuck* pc_epuck, int n_write_to_file) : CController (pch_name, pc_epuck)

{
	/* Set Write to File */
	m_nWriteToFile = n_write_to_file;
	
	/* Set epuck */
	m_pcEpuck = pc_epuck;
	/* Set Wheels */
	m_acWheels = (CWheelsActuator*) m_pcEpuck->GetActuator(ACTUATOR_WHEELS);
	/* Set Prox Sensor */
	m_seProx = (CEpuckProximitySensor*) m_pcEpuck->GetSensor(SENSOR_PROXIMITY);
	/* Set light Sensor */
	m_seLight = (CRealLightSensor*) m_pcEpuck->GetSensor(SENSOR_REAL_LIGHT);
	/* Set blue light Sensor */
	m_seBlueLight = (CRealBlueLightSensor*) m_pcEpuck->GetSensor(SENSOR_REAL_BLUE_LIGHT);
	/* Set contact Sensor */
	m_seContact = (CContactSensor*) m_pcEpuck->GetSensor (SENSOR_CONTACT);
	/* Set ground Sensor */
	m_seGround = (CGroundSensor*) m_pcEpuck->GetSensor (SENSOR_GROUND);
	/* Set ground memory Sensor */
	m_seGroundMemory = (CGroundMemorySensor*) m_pcEpuck->GetSensor (SENSOR_GROUND_MEMORY);
	/* Set battery Sensor */
	m_seBattery = (CBatterySensor*) m_pcEpuck->GetSensor (SENSOR_BATTERY);

	
	/* Initilize Variables */
	m_fLeftSpeed = 0.0;
	m_fRightSpeed = 0.0;

	fBattToGoTableInhibitor = 1.0;
	num_Platos = 0;


	/* Create TABLE for the COORDINATOR */
	m_fActivationTable = new double* [BEHAVIORS];
	for ( int i = 0 ; i < BEHAVIORS ; i++ )
	{
		m_fActivationTable[i] = new double[3];
	}
}

/******************************************************************************/
/******************************************************************************/

CIri1Controller::~CIri1Controller()
{
	for ( int i = 0 ; i < BEHAVIORS ; i++ )
	{
		delete [] m_fActivationTable;
	}
}


/******************************************************************************/
/******************************************************************************/

void CIri1Controller::SimulationStep(unsigned n_step_number, double f_time, double f_step_interval)
{
	
	/* Move time to global variable, so it can be used by the bahaviors to write to files*/
	m_fTime = f_time;

	/* Execute the levels of competence */
	ExecuteBehaviors();

	/* Execute Coordinator */
	Coordinator();

	/* Set Speed to wheels */
	m_acWheels->SetSpeed(m_fLeftSpeed, m_fRightSpeed);


	if (m_nWriteToFile ) 
	{
	/* INIT: WRITE TO FILES */
	/* Write robot position and orientation */
		FILE* filePosition = fopen("outputFiles/robotPosition", "a");
		fprintf(filePosition,"%2.4f %2.4f %2.4f %2.4f\n", m_fTime, m_pcEpuck->GetPosition().x, m_pcEpuck->GetPosition().y, m_pcEpuck->GetRotation());
		fclose(filePosition);

		/* Write robot wheels speed */
		FILE* fileWheels = fopen("outputFiles/robotWheels", "a");
		fprintf(fileWheels,"%2.4f %2.4f %2.4f \n", m_fTime, m_fLeftSpeed, m_fRightSpeed);
		fclose(fileWheels);
		/* END WRITE TO FILES */
	}
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::ExecuteBehaviors ( void )
{
	for ( int i = 0 ; i < BEHAVIORS ; i++ )
	{
		m_fActivationTable[i][2] = 0.0;
	}

	/* Release Inhibitors */
	fBattToGoTableInhibitor = 1.0;
	/* Set Leds to BLACK */
	m_pcEpuck->SetAllColoredLeds(	LED_COLOR_BLACK);
	
	ObstacleAvoidance ( AVOID_PRIORITY );
  	GoLoad ( RELOAD_PRIORITY );	
	GoRest( REST_PRIORITY );
	GoTable ( TABLE_PRIORITY );
	Stop( STOP_PRIORITY );
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::Coordinator ( void )
{
  int nBehavior;
  double fAngle = 0.0;

  int nActiveBehaviors = 0;
  
  /* Create vector of movement */
  dVector2  vAngle;
  vAngle.x = 0.0;
  vAngle.y = 0.0;
  
  /* For every Behavior */
	for ( nBehavior = 0 ; nBehavior < BEHAVIORS -1 ; nBehavior++ )
	{
    /* If behavior is active */
		if ( m_fActivationTable[nBehavior][2] == 1.0 )
		{
      /* DEBUG */
      printf("Behavior %d: %2f\n", nBehavior, m_fActivationTable[nBehavior][0]);
      /* DEBUG */
      vAngle.x += m_fActivationTable[nBehavior][1] * cos(m_fActivationTable[nBehavior][0]);
      vAngle.y += m_fActivationTable[nBehavior][1] * sin(m_fActivationTable[nBehavior][0]);
		}
	}

  /* Calc angle of movement */
  fAngle = atan2(vAngle.y, vAngle.x);
  
  /* Normalize fAngle */
  while ( fAngle > M_PI ) fAngle -= 2 * M_PI;
	while ( fAngle < -M_PI ) fAngle += 2 * M_PI;
 
  /* Based on the angle, calc wheels movements */
  double fCLinear = 1.0;
  double fCAngular = 1.0;
  double fC1 = SPEED / M_PI;

  /* Calc Linear Speed */
  double fVLinear = SPEED * fCLinear * ( cos ( fAngle / 2) );

  /*Calc Angular Speed */
  double fVAngular = fAngle;

 /* Si el flag de Stop esta activado se para el robot, sino la velocidad de sus ruedad es el resultado del sumatorio de las velocidades calculadas para flag activo excepto Stop */
  if (m_fActivationTable[STOP_PRIORITY][2] == 1.0){

	printf("Behavior %d: %2f\n", STOP_PRIORITY, m_fActivationTable[STOP_PRIORITY][0]);

	m_fLeftSpeed  = m_fActivationTable[STOP_PRIORITY][0];
  	m_fRightSpeed = m_fActivationTable[STOP_PRIORITY][1];
  }else {
	m_fLeftSpeed  = fVLinear - fC1 * fVAngular;
  	m_fRightSpeed = fVLinear + fC1 * fVAngular;
  }

  //printf("Numero de Platos : %d \n", num_Platos);
  printf("LEFT: %2f, RIGHT: %2f\n\n", m_fLeftSpeed, m_fRightSpeed);
	if (m_nWriteToFile ) 
	{
		/* INIT: WRITE TO FILES */
		/* Write coordinator ouputs */
		FILE* fileOutput = fopen("outputFiles/coordinatorOutput", "a");
		fprintf(fileOutput,"%2.4f %2.4f %2.4f %2.4f %2.4f %2.4f \n", m_fTime, m_fActivationTable[AVOID_PRIORITY][2], m_fActivationTable[RELOAD_PRIORITY][2], m_fActivationTable[REST_PRIORITY][2], m_fActivationTable[TABLE_PRIORITY][2], m_fActivationTable[STOP_PRIORITY][2]);
		fclose(fileOutput);
		/* END WRITE TO FILES */
	}
	
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::ObstacleAvoidance ( unsigned int un_priority )
{
	
	/* Leer Sensores de Proximidad */
	double* prox = m_seProx->GetSensorReading(m_pcEpuck);

	double fMaxProx = 0.0;
	const double* proxDirections = m_seProx->GetSensorDirections();

	dVector2 vRepelent;
	vRepelent.x = 0.0;
	vRepelent.y = 0.0;

	printf("Prox Sensor Value: ");
	for ( int i = 0 ; i < m_seProx->GetNumberOfInputs() ; i++)
	{
		printf("%2f ",prox[i]);
	}
	printf("\n");

	/* Calc vector Sum */
	for ( int i = 0 ; i < m_seProx->GetNumberOfInputs() ; i ++ )
	{
		vRepelent.x += prox[i] * cos ( proxDirections[i] );
		vRepelent.y += prox[i] * sin ( proxDirections[i] );

		if ( prox[i] > fMaxProx )
			fMaxProx = prox[i];
	}
	
	/* Calc pointing angle */
	float fRepelent = atan2(vRepelent.y, vRepelent.x);
	/* Create repelent angle */
	fRepelent -= M_PI;
	/* Normalize angle */
	while ( fRepelent > M_PI ) fRepelent -= 2 * M_PI;
	while ( fRepelent < -M_PI ) fRepelent += 2 * M_PI;

  m_fActivationTable[un_priority][0] = fRepelent;
  m_fActivationTable[un_priority][1] = fMaxProx;

	/* If above a threshold */
	if ( fMaxProx > PROXIMITY_THRESHOLD )
	{
		/* Set Leds to GREEN */
		m_pcEpuck->SetAllColoredLeds(	LED_COLOR_GREEN);
    /* Mark Behavior as active */
    m_fActivationTable[un_priority][2] = 1.0;
	}
	
	if (m_nWriteToFile ) 
	{
		/* INIT WRITE TO FILE */
		/* Write level of competence ouputs */
		FILE* fileOutput = fopen("outputFiles/avoidOutput", "a");
		fprintf(fileOutput, "%2.4f %2.4f ", m_fTime, fMaxProx);
		fprintf(fileOutput, "%2.4f\n",m_fActivationTable[un_priority][2]);
		fclose(fileOutput);
		/* END WRITE TO FILE */
	}
}

/******************************************************************************/
/******************************************************************************/

/* Este comportamiento hace que el robot se pare */
void CIri1Controller::Stop ( unsigned int un_priority )
{
    /* Leer Sensores de Suelo*/
	double* ground = m_seGround->GetSensorReading(m_pcEpuck);

	/* Leer Sensores de Luz Azul*/
	double* bluelight = m_seBlueLight->GetSensorReading(m_pcEpuck);

	double sumGround;
	double sumBlueLight;

	/* Sumar los valores del sensor de luz azul */
	for ( int i = 0 ; i < m_seBlueLight->GetNumberOfInputs() ; i++ )
	{
		sumBlueLight += bluelight[i];
	}

	/* Sumar los valores del sensor del suelo */
	for ( int i = 0 ; i < m_seGround->GetNumberOfInputs() ; i ++ )
	{
		sumGround += ground[i];
	}

	printf("Suma Valor Suelo: %2.4f %2.4f\n", sumGround, ground[0]);

	/* Epsilon declarada para evitar cualquier problema que pueda tener c++ al comparar valores double que sean 0.0 ambos*/
    double epsilon = 1e-9;
	
	/* Si esta sobre la alfombrilla negra descarga los platos, 
	ademas si estan todas las luces azules apagadas se activa el flag */
	if (sumGround < epsilon) {
		num_Platos = 0;
		if (sumBlueLight < epsilon){

			printf("Esperando\n");
			m_fActivationTable[un_priority][2] = 1.0;
			m_pcEpuck->SetAllColoredLeds(	LED_COLOR_WHITE);
		}

	}

	m_fActivationTable[un_priority][0] = STOP_SPEED;
  	m_fActivationTable[un_priority][1] = STOP_SPEED;

	if (m_nWriteToFile ) 
	{
		/* INIT: WRITE TO FILES */
		/* Write level of competence ouputs */
		FILE* fileOutput = fopen("outputFiles/stopOutput", "a");
		fprintf(fileOutput,"%2.4f %2.4f %2.4f %2.4f \n", m_fTime, m_fActivationTable[un_priority][2], sumGround, sumBlueLight);
		fclose(fileOutput);
		/* END WRITE TO FILES */
	}

}
		
/******************************************************************************/
/******************************************************************************/

/* Este comportamiento hace que el robot vaya a las mesas en las que hay una luz encendida */
void CIri1Controller::GoTable ( unsigned int un_priority )
{
	/* Leer Battery Sensores */
	double* battery = m_seBattery->GetSensorReading(m_pcEpuck);

	/* Leer Sensores de Luz Azul*/
	double* bluelight = m_seBlueLight->GetSensorReading(m_pcEpuck);


	double fMaxLight = 0.0;
	const double* lightBlueDirections = m_seBlueLight->GetSensorDirections();


	printf("Light Blue Sensor Value: ");
	for ( int i = 0 ; i < m_seBlueLight->GetNumberOfInputs() ; i++)
	{
		printf("%2f ", bluelight[i]);
	}
	printf("\n");

  /* We call vRepelent to go similar to Obstacle Avoidance, although it is an aproaching vector */
	dVector2 vRepelent;
	vRepelent.x = 0.0;
	vRepelent.y = 0.0;

	/* Calc vector Sum */
	for ( int i = 0 ; i < m_seProx->GetNumberOfInputs() ; i ++ ){

		vRepelent.x += bluelight[i] * cos ( lightBlueDirections[i] );
		vRepelent.y += bluelight[i] * sin ( lightBlueDirections[i] );

		if ( bluelight[i] > fMaxLight )
			fMaxLight = bluelight[i];
	}
	printf("%2f ", fMaxLight);
	printf("\n");

	/* Calc pointing angle */
	float fRepelent = atan2(vRepelent.y, vRepelent.x);
	
  /* Normalize angle */
	while ( fRepelent > M_PI ) fRepelent -= 2 * M_PI;
	while ( fRepelent < -M_PI ) fRepelent += 2 * M_PI;


  m_fActivationTable[un_priority][0] = fRepelent;
  m_fActivationTable[un_priority][1] = fMaxLight * 0.95;

	/* Si los sensores detectan luz azul y el robot tiene menos de tres platos activa el flag */
	if ( fMaxLight > 0 && num_Platos < 3 )
	{
		/* Set Leds to BLUE */
		m_pcEpuck->SetAllColoredLeds(	LED_COLOR_BLUE);
		
		/* Activa el flag, pero es dependera de si el inhibidor de la bateria esta activo o no */
		m_fActivationTable[un_priority][2] = 1.0 * fBattToGoTableInhibitor;
	}	

	double sumBlueLight;
	for ( int i = 0 ; i < 8 ; i ++ ){
		sumBlueLight += bluelight[i];
	}

	/* Si la suma de todos los sensores supera el umbral, apagara la luz, ademas anade un plato al contador de platos */
	if ( sumBlueLight > 1.67 ){	

		m_seBlueLight->SwitchNearestLight(0);
		num_Platos = num_Platos +1;
	}	


	if (m_nWriteToFile ) 
	{
		/* INIT WRITE TO FILE */
		FILE* fileOutput = fopen("outputFiles/tableOutput", "a");
		fprintf(fileOutput, "%2.4f %2.4f %2.4f %d ", m_fTime, battery[0], sumBlueLight, num_Platos);
		fprintf(fileOutput, "%2.4f \n",m_fActivationTable[un_priority][2]);
		fclose(fileOutput);
		/* END WRITE TO FILE */
	}
}

		
/******************************************************************************/
/******************************************************************************/

/* Este comportamiento hace que el robot vaya a cargar la bateria */
void CIri1Controller::GoLoad ( unsigned int un_priority )
{
	/* Leer Battery Sensores */
	double* battery = m_seBattery->GetSensorReading(m_pcEpuck);

	/* Leer Sensores de Luz */
	double* light = m_seLight->GetSensorReading(m_pcEpuck);

	double fMaxLight = 0.0;
	const double* lightDirections = m_seLight->GetSensorDirections();

	printf("Light Sensor Value: ");
	for ( int i = 0 ; i < m_seLight->GetNumberOfInputs() ; i++)
	{
		printf("%2f ", light[i]);
	}
	printf("\n");

	printf(" BATTERY: %1.3f \n", battery[0]);

  /* We call vRepelent to go similar to Obstacle Avoidance, although it is an aproaching vector */
	dVector2 vRepelent;
	vRepelent.x = 0.0;
	vRepelent.y = 0.0;

	/* Calc vector Sum */
	for ( int i = 0 ; i < m_seProx->GetNumberOfInputs() ; i ++ )
	{
		vRepelent.x += light[i] * cos ( lightDirections[i] );
		vRepelent.y += light[i] * sin ( lightDirections[i] );

		if ( light[i] > fMaxLight )
			fMaxLight = light[i];
	}
	
	/* Calc pointing angle */
	float fRepelent = atan2(vRepelent.y, vRepelent.x);
	
  /* Normalize angle */
	while ( fRepelent > M_PI ) fRepelent -= 2 * M_PI;
	while ( fRepelent < -M_PI ) fRepelent += 2 * M_PI;


  m_fActivationTable[un_priority][0] = fRepelent;
  m_fActivationTable[un_priority][1] = fMaxLight; //0.2;

	/* If battery below a BATTERY_THRESHOLD */
	if ( battery[0] < BATTERY_THRESHOLD )
	{
		/* Inibit GoTable */
		fBattToGoTableInhibitor = 0.0;

		/* Set Leds to RED */
		m_pcEpuck->SetAllColoredLeds(	LED_COLOR_RED);

		 //m_fActivationTable[un_priority][0] = fRepelent;
		
		/* Mark behavior as active */
		m_fActivationTable[un_priority][2] = 1.0;
	}

	if (m_nWriteToFile ) 
	{
		/* INIT WRITE TO FILE */
		FILE* fileOutput = fopen("outputFiles/batteryOutput", "a");
		fprintf(fileOutput, "%2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f ", m_fTime, battery[0], light[0], light[1], light[2], light[3], light[4], light[5], light[6], light[7]);
		fprintf(fileOutput, "%2.4f %2.4f %2.4f\n",m_fActivationTable[un_priority][2], m_fActivationTable[un_priority][0], m_fActivationTable[un_priority][1]);
		fclose(fileOutput);
		/* END WRITE TO FILE */
	}
	
}

/******************************************************************************/
/******************************************************************************/

/* Este comportamiento hace que el robot vaya al punto de descarga */
void CIri1Controller::GoRest ( unsigned int un_priority )
{
	
	printf("Numero de Platos : %d \n", num_Platos);

	/* Leer Sensores de Luz */
	double* light = m_seLight->GetSensorReading(m_pcEpuck);

	/* Leer Sensores de Luz Azul*/
	double* bluelight = m_seBlueLight->GetSensorReading(m_pcEpuck);

	double sumBlueLight = 0;

	/* Sumar los valores del sensor de luz azul */
	for ( int i = 0 ; i < m_seBlueLight->GetNumberOfInputs() ; i++ )
	{
		sumBlueLight += bluelight[i];
	}
	
	double fMaxLight = 0.0;
	const double* lightDirections = m_seLight->GetSensorDirections();

  /* We call vRepelent to go similar to Obstacle Avoidance, although it is an aproaching vector */
	dVector2 vRepelent;
	vRepelent.x = 0.0;
	vRepelent.y = 0.0;

	/* Calc vector Sum */
	for ( int i = 0 ; i < m_seProx->GetNumberOfInputs() ; i ++ )
	{
		vRepelent.x += light[i] * cos ( lightDirections[i] );
		vRepelent.y += light[i] * sin ( lightDirections[i] );

		if ( light[i] > fMaxLight )
			fMaxLight = light[i];
	}
	
	/* Calc pointing angle */
	float fRepelent = atan2(vRepelent.y, vRepelent.x);
	
    /* Normalize angle */
	while ( fRepelent > M_PI ) fRepelent -= 2 * M_PI;
	while ( fRepelent < -M_PI ) fRepelent += 2 * M_PI;

  m_fActivationTable[un_priority][0] = fRepelent;
  m_fActivationTable[un_priority][1] = fMaxLight * 0.95;
	
	/* Epsilon declarada para evitar cualquier problema que pueda tener c++ al comparar valores double que sean 0.0 ambos*/
	double epsilon = 1e-9;

  /* Si lleva mas de dos platos o no hay luces azules encendidas activa el flag */
	if ( num_Platos > 2 || sumBlueLight < epsilon )
	{
		/* Set Leds to YELLOW */
		m_pcEpuck->SetAllColoredLeds(	LED_COLOR_YELLOW);
    /* Mark Behavior as active */
    m_fActivationTable[un_priority][2] = 1.0;
		
	}

	if (m_nWriteToFile ) 
	{
		/* INIT WRITE TO FILE */
		FILE* fileOutput = fopen("outputFiles/gorestOutput", "a");
		fprintf(fileOutput, "%2.4f %2.4f %d ", m_fTime, sumBlueLight, num_Platos);
		fprintf(fileOutput, "%2.4f %2.4f %2.4f\n",m_fActivationTable[un_priority][2]);
		fclose(fileOutput);
		/* END WRITE TO FILE */
	}


}
