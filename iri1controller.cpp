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
#define FORAGE_PRIORITY		2
#define TABLE_PRIORITY		3
#define STOP_PRIORITY 		4


#define PROXIMITY_THRESHOLD 0.55
#define BATTERY_THRESHOLD 0.3

#define SPEED 300.0


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

	fBattToForageInhibitor = 1.0;


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
	/* Set Leds to BLACK */
	m_pcEpuck->SetAllColoredLeds(	LED_COLOR_BLACK);
	
	ObstacleAvoidance ( AVOID_PRIORITY );
  	GoLoad ( RELOAD_PRIORITY );	
	Forage( FORAGE_PRIORITY );
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
	for ( nBehavior = 0 ; nBehavior < BEHAVIORS ; nBehavior++ )
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

  m_fLeftSpeed  = fVLinear - fC1 * fVAngular;
  m_fRightSpeed = fVLinear + fC1 * fVAngular;

  printf("LEFT: %2f, %2f\n", m_fLeftSpeed, m_fRightSpeed);
	if (m_nWriteToFile ) 
	{
		/* INIT: WRITE TO FILES */
		/* Write coordinator ouputs */
		FILE* fileOutput = fopen("outputFiles/coordinatorOutput", "a");
		fprintf(fileOutput,"%2.4f %d %2.4f %2.4f \n", m_fTime, nBehavior, m_fLeftSpeed, m_fRightSpeed);
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
		fprintf(fileOutput, "%2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f ", m_fTime, prox[0], prox[1], prox[2], prox[3], prox[4], prox[5], prox[6], prox[7], fMaxProx, fRepelent);
		fprintf(fileOutput, "%2.4f %2.4f %2.4f\n",m_fActivationTable[un_priority][2], m_fActivationTable[un_priority][0], m_fActivationTable[un_priority][1]);
		fclose(fileOutput);
		/* END WRITE TO FILE */
	}
}

/******************************************************************************/
/******************************************************************************/


void CIri1Controller::Stop ( unsigned int un_priority )
{
  /* Direction Angle 0.0 and always active. We set its vector intensity to 0.5 if used */
	m_fActivationTable[un_priority][0] = 0.0;
	m_fActivationTable[un_priority][1] = 0.5;
	m_fActivationTable[un_priority][2] = 1.0;

	m_fLeftSpeed = 0.0;
	m_fRightSpeed = 0.0;

	//No para porque el coordinador mantiene el movimiento

	if (m_nWriteToFile ) 
	{
		/* INIT: WRITE TO FILES */
		/* Write level of competence ouputs */
		FILE* fileOutput = fopen("outputFiles/navigateOutput", "a");
		fprintf(fileOutput,"%2.4f %2.4f %2.4f %2.4f \n", m_fTime, m_fActivationTable[un_priority][2], m_fActivationTable[un_priority][0], m_fActivationTable[un_priority][1]);
		fclose(fileOutput);
		/* END WRITE TO FILES */
	}

}
		
/******************************************************************************/
/******************************************************************************/

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
  m_fActivationTable[un_priority][1] = fMaxLight;

	/* If battery below a BATTERY_THRESHOLD */
	if ( fMaxLight > 0 )
	{
		/* Set Leds to RED */
		m_pcEpuck->SetAllColoredLeds(	LED_COLOR_BLUE);
		
		/* Mark behavior as active */
		m_fActivationTable[un_priority][2] = 1.0;
	}	

	double sumBlueLight;
	for ( int i = 0 ; i < 8 ; i ++ ){
		sumBlueLight += bluelight[i];
	}

	if ( sumBlueLight > 1.7 ){	

		m_seBlueLight->SwitchNearestLight(0);
	}	


	if (m_nWriteToFile ) 
	{
		/* INIT WRITE TO FILE */
		FILE* fileOutput = fopen("outputFiles/tableOutput", "a");
		fprintf(fileOutput, "%2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f ", m_fTime, battery[0], bluelight[0], bluelight[1], bluelight[2], bluelight[3], bluelight[4], bluelight[5], bluelight[6], bluelight[7]);
		fprintf(fileOutput, "%2.4f %2.4f %2.4f\n",m_fActivationTable[un_priority][2], m_fActivationTable[un_priority][0], m_fActivationTable[un_priority][1]);
		fclose(fileOutput);
		/* END WRITE TO FILE */
	}
}

		
/******************************************************************************/
/******************************************************************************/

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
		/* Inibit Forage */
		fBattToForageInhibitor = 0.0;

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

void CIri1Controller::Forage ( unsigned int un_priority )
{
	/* Leer Sensores de Suelo Memory */
	double* groundMemory = m_seGroundMemory->GetSensorReading(m_pcEpuck);
	
	/* Leer Sensores de Luz */
	double* light = m_seLight->GetSensorReading(m_pcEpuck);
	
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
	/* Create repelent angle */
	//fRepelent -= M_PI;
	
  /* Normalize angle */
	while ( fRepelent > M_PI ) fRepelent -= 2 * M_PI;
	while ( fRepelent < -M_PI ) fRepelent += 2 * M_PI;

  m_fActivationTable[un_priority][0] = fRepelent;
  m_fActivationTable[un_priority][1] = fMaxLight;
  
  /* If with a virtual puck */
	if ( ( groundMemory[0] * fBattToForageInhibitor ) == 1.0 )
	{
		/* Set Leds to BLUE */
		m_pcEpuck->SetAllColoredLeds(	LED_COLOR_YELLOW);
    /* Mark Behavior as active */
    m_fActivationTable[un_priority][2] = 1.0;
		
	}

	if (m_nWriteToFile ) 
	{
		/* INIT WRITE TO FILE */
		FILE* fileOutput = fopen("outputFiles/forageOutput", "a");
		fprintf(fileOutput, "%2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f ", m_fTime, fBattToForageInhibitor, groundMemory[0], light[0], light[1], light[2], light[3], light[4], light[5], light[6], light[7]);
		fprintf(fileOutput, "%2.4f %2.4f %2.4f\n",m_fActivationTable[un_priority][2], m_fActivationTable[un_priority][0], m_fActivationTable[un_priority][1]);
		fclose(fileOutput);
		/* END WRITE TO FILE */
	}


}
