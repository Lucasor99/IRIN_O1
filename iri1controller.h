#ifndef IRI1CONTROLLER_H_
#define IRI1CONTROLLER_H_


/******************************************************************************/
/******************************************************************************/

#include "controller.h"

/******************************************************************************/
/******************************************************************************/

class CIri1Controller : public CController
{
public:

    CIri1Controller (const char* pch_name, CEpuck* pc_epuck, int n_write_to_file);
    ~CIri1Controller();
    void SimulationStep(unsigned n_step_number, double f_time, double f_step_interval);

private:
    CEpuck* m_pcEpuck;
    
		CWheelsActuator* m_acWheels;
    CEpuckProximitySensor* m_seProx;
		CRealLightSensor* m_seLight;
		CRealBlueLightSensor* m_seBlueLight;
		CRealRedLightSensor* m_seRedLight;
		CContactSensor* m_seContact;
		CGroundSensor* m_seGround;
		CGroundMemorySensor* m_seGroundMemory;
		CBatterySensor* m_seBattery;  
		CBlueBatterySensor* m_seBlueBattery;  
		CRedBatterySensor* m_seRedBattery;  
		CEncoderSensor* m_seEncoder;  
		CCompassSensor* m_seCompass;  

    float m_fOrientation; 
    dVector2 m_vPosition;
	vector<CBlueLightObject*> m_vecBlueLightObject;
		

		/* Global Variables */
		double 		m_fLeftSpeed;
		double 		m_fRightSpeed;
		double**	m_fActivationTable;
		double 		m_fTime;
		double		fBattToForageInhibitor;
		double		fBattToGoTableInhibitor;
		double		fStopInhibitor;
		int			num_Platos;

		int m_nWriteToFile;
		/* Functions */

		void ExecuteBehaviors ( void );
		void Coordinator ( void );

		void ObstacleAvoidance ( unsigned int un_priority );
		void GoLoad ( unsigned int un_priority );
		void GoTable ( unsigned int un_priority );
		void GoRest ( unsigned int un_priority );
		void Stop ( unsigned int un_priority );
};

#endif
