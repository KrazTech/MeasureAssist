void servoInit( void )
{
   // Setup LidarServo
	LidarServo.EPwmRegHandle 	= &EPwm2Regs;
	LidarServo.CaptureValue	= 0;
	LidarServo.angle		= 0.0;
	LidarServo.flags		= 0x00;

   // Setup TBCLK
   // Make sure 200MHz clock is divided by 2.
   ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV = 0x1;  
   LidarServo.EPwmRegHandle->TBCTL.bit.CTRMODE = TB_COUNT_UP;// Count up
   LidarServo.EPwmRegHandle->TBPRD = SERVO_PRD;              // Set timer period
   LidarServo.EPwmRegHandle->TBCTL.bit.PHSEN = TB_DISABLE;   // Disable phase loading
   LidarServo.EPwmRegHandle->TBPHS.bit.TBPHS = 0x0000;       // Phase is 0
   LidarServo.EPwmRegHandle->TBCTR = 0x0000;                 // Clear counter
   // Clock ratio to SYSCLKOUT   
   LidarServo.EPwmRegHandle->TBCTL.bit.HSPCLKDIV = TB_DIV1;   
   // 101 sets to div 32. input clk = 3.125MHz
   LidarServo.EPwmRegHandle->TBCTL.bit.CLKDIV = 0x5;				
   // Setup shadow register load on ZERO
   LidarServo.EPwmRegHandle->CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   LidarServo.EPwmRegHandle->CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   LidarServo.EPwmRegHandle->CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
   LidarServo.EPwmRegHandle->CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

   // Set default angle
   LidarServo.angle = 90.0;
   setServoAngle();

   // Set actions
   LidarServo.EPwmRegHandle->AQCTLA.bit.ZRO = AQ_SET;   // Set PWM1A on Zero
   LidarServo.EPwmRegHandle->AQCTLA.bit.CAU = AQ_CLEAR; // Clr A on event A, up count
   LidarServo.EPwmRegHandle->AQCTLB.bit.ZRO = AQ_SET;   // Set B on Zero
   LidarServo.EPwmRegHandle->AQCTLB.bit.CBU = AQ_CLEAR; // Clr B on event B, up count
   return;
}


void setServoAngle( void ) {

		Uint16 tempangle;
		tempangle = (Uint16) LidarServo.angle;
		switch (tempangle) {
		case 30:
			LidarServo.CaptureValue = angle30;
			break;
		case 40:
			LidarServo.CaptureValue = angle40;
			break;
		case 50:
			LidarServo.CaptureValue = angle50;
			break;
		case 60:
			LidarServo.CaptureValue = angle60;
			break;
		case 70:
			LidarServo.CaptureValue = angle70;
			break;
		case 80:
			LidarServo.CaptureValue = angle80;
			break;
		case 90:
			LidarServo.CaptureValue = angle90;
			break;
		case 100:
			LidarServo.CaptureValue = angle100;
			break;
		case 110:
			LidarServo.CaptureValue = angle110;
			break;
		case 120:
			LidarServo.CaptureValue = angle120;
			break;
		case 130:
			LidarServo.CaptureValue = angle130;
			break;
		case 140:
			LidarServo.CaptureValue = angle140;
			break;
		case 150:
			LidarServo.CaptureValue = angle150;
			break;
		default:
			LidarServo.CaptureValue = angle90;
			break;
		}
		// Set Compare values
LidarServo.EPwmRegHandle->CMPA.bit.CMPA = LidarServo.CaptureValue; // Set cmp A value
LidarServo.EPwmRegHandle->CMPB.bit.CMPB = LidarServo.CaptureValue; // Set cmp B value	return;
}


void lidarInit() {

	Uint16 write[1];

	write[0] = 0x00;
	I2C_Write(LIDAR_ADDRESS, 0x00, 1, write);			// Reset.
}


double lidarDistance() {
	Uint16 write[1];
	Uint16 result[2];
	double sum;
	sum = 0;
	char i;
	result [0] = 0x0000;
	result [1] = 0x0000;

	for (i = 0; i < 8; i++) {
		write[0] = 0x04;
             
// Take acquisition & correlation processing with DC correction
		I2C_Write(LIDAR_ADDRESS, 0x00, 1, write);					
		
		g_TICK_EVENTS.g_DELAY = g_TICK_EVENTS.TickCount + 4;	    // delay 20 ms.
		while (g_TICK_EVENTS.g_DELAY > g_TICK_EVENTS.TickCount);// wait

             // Read the measurement
		I2C_Read(LIDAR_ADDRESS, 0x8F, 2, result);	   
		sum += (((result[0] << 8) | result[1]) + (LIDAR_OFFSET));
	}

	sum /= 8;

	return sum;					// return the measurement result.
}


void offsetAdjust(mPoint *pSample) {
	double bAngle, bDistance;
	double bufferA, bufferB, bufferC, bufferD, bufferE, bufferF;
	bAngle = (*pSample).angle;
	bDistance = (*pSample).distance;

	if (bAngle < 78.0f) {

		// Adjust the sample so the reference is from the projection axis.
		bufferC = tan((bAngle / 360.0f) * TWO_PI) * projectXoffset;   //theta
		bufferD = projectXoffset / (cos((bAngle / 360.0f) * TWO_PI)); //weirdn

		bufferE = bDistance - bufferD;					// aPrime
		bufferF = projectYoffset - bufferC;				// s.phi

		//thetaPrime
		(*pSample).distance = sqrt((bufferE * bufferE) + (bufferF * bufferF) - (2.0f * bufferE * bufferF * cos (((90.0f - bAngle) / 360.0f) * TWO_PI)));
		// angleA calculation
		bufferA = (bufferE * sin(((90.0f - bAngle) / 360.0f) * TWO_PI));
		bufferB = asin( bufferA / (*pSample).distance);
		bufferC = ((bufferB / TWO_PI) * 360.0f );

		if ((bufferC - 90.0f) < 0) {
			bufferF = (bufferC - 90.0f) * (-1.0f);
		} else {
			bufferF = bufferC - 90.0f;
		}
		(*pSample).angle = bufferF; // save it

	} else {

		bufferA = bAngle - 78.6901f;          // Theta value for cosine law
		
		// Adjusted measurement
bufferB = sqrt((bDistance * bDistance) + (2.54951f * 2.54951f) - (2 * bDistance * 2.54951f * cos( (bufferA / 360.0f) * TWO_PI )));	
		
// Calculate intermediate angle A for adjusted angle
bufferC = ( asin((bDistance * sin((bufferA / 360.0f) * TWO_PI )) / bufferB ) / TWO_PI ) * 360.0f;							

		(*pSample).distance = bufferB;

		if (bufferC < 90) {
			(*pSample).angle = 258.69f - (180.0f - bufferC);
		} else {
			(*pSample).angle = 258.69f - bufferC;
		}
	}
}


void StartScan( void )
{
	int i;
	for (i = 0; i < SCAN_LIMIT; i++)
	{
		MeasurementBuffer[i] = 0;
	}
	for (i = 0; i < servoSteps; i++) {
			testMeasurements[i].angle = 0;
			testMeasurements[i].distance = 0;
	}

	// Set the servo Angle to 0
	LidarServo.angle = g_Offset;
// Update the Servo angle						
	setServoAngle();
	// Stamp the Next event time.									g_TICK_EVENTS.Scan_TS = g_TICK_EVENTS.TickCount + FIRSTDELAY;		
// Set the scanning flag	
ScanFlags |= SCAN_EN;									

}

void ScanEvent( void ){

	int startPoint, stopPoint;
	startPoint = 0;
	stopPoint = 1;
	double m_measSum;
	double c, angleB, bPrime, thetaPrime;
	double bufferA, bufferB, bufferC, bufferD;
	c = 0.0;
	angleB = 0.0;
	bPrime = 0.0;
	thetaPrime = 0.0;

// Check to see if it's time to handle the event.
if (g_TICK_EVENTS.Scan_TS <= g_TICK_EVENTS.TickCount && (ScanFlags & SCAN_EN) == 1)
{

m_measSum = 0;					// just surface summation
	testMeasurements[0].distance = lidarDistance();
	testMeasurements[0].angle = LidarServo.angle;
	offsetAdjust(&testMeasurements[0]);

	while (m_measSum < sys_Measurement) {

		LidarServo.angle += servoResolution;
		setServoAngle();
		g_TICK_EVENTS.g_DELAY = g_TICK_EVENTS.TickCount + MOVEDELAY;
		while (g_TICK_EVENTS.TickCount < g_TICK_EVENTS.g_DELAY);
		// Adjust the sample so the reference is from the projection axis.
		testMeasurements[stopPoint].distance = lidarDistance();
		testMeasurements[stopPoint].angle = LidarServo.angle;
		offsetAdjust(&testMeasurements[stopPoint]);

		// Calculate Surface between measurements
bufferA =  testMeasurements[startPoint].distance * testMeasurements[startPoint].distance ;
		
bufferB =  testMeasurements[stopPoint].distance * testMeasurements[stopPoint].distance ;
				
bufferC = 2 * testMeasurements[startPoint].distance * testMeasurements[stopPoint].distance ;
				
bufferD = cos(((testMeasurements[stopPoint].angle - testMeasurements[startPoint].angle) / 360.0f) * TWO_PI);
				
c = sqrt( (bufferA) + ( bufferB ) - (bufferC * bufferD));

		// check to see if the measurement ends within this segment

		if ((sys_Measurement - m_measSum) <= c) {

			// Calculate angleB
			bufferA = (testMeasurements[stopPoint].distance * sin((servoResolution / 360.0f) * TWO_PI));

			bufferB = bufferA / c;
			angleB = asin( bufferB );

			// Calculate bPrime, what b should be for the desired measurement
			bufferA = (sys_Measurement - m_measSum) * (sys_Measurement - m_measSum);

			bufferB = testMeasurements[startPoint].distance * testMeasurements[stopPoint].distance;

			bufferC = 2 * testMeasurements[startPoint].distance * (sys_Measurement - m_measSum);

			bufferD = cos(angleB);                                           

			bPrime = sqrt( ( bufferA ) + ( bufferB ) - (bufferC * bufferD));

			//Calculate the dAngle for the ssetpoint
			bufferA = sin(angleB);
			bufferB = ((sys_Measurement - m_measSum) * bufferA);
			bufferC = bufferB / bPrime;
			thetaPrime += (asin( bufferC ) / TWO_PI) * 360;		
		} else {
// Increment the starting measurement
			startPoint++;	
// Increment the ending measurement
			stopPoint++;
// add the servo resolution to the setpoint angle.
thetaPrime += servoResolution;						
		}
// Summate the surface length with this segment.
		m_measSum += c;																	
	}

// Set the angle for turning off the laser.
laserPulse[1] = (Uint32) ((testMeasurements[0].angle + thetaPrime) * (45.5111));
// Set the angle to turn the laser ON
laserPulse[0] = (Uint32) ((testMeasurements[0].angle) * (45.5111));							
ScanFlags &= ~SCAN_EN;									
// Clear the Scan Flag
ScanFlags |= SCAN_FIN;

}


void motorInit(void) {
EQep1Regs.QUPRD=0x000000ff;	// Unit Timer for 100Hz at 200 MHz SYSCLKOUT

EQep1Regs.QEINT.bit.PCM = 1;	// Turn on interrupt Position Compare
EQep1Regs.QDECCTL.bit.QSRC=00;	// QEP quadrature count mode
EQep1Regs.QDECCTL.bit.QAP=0;	// Invert Pulse on QA
EQep1Regs.QDECCTL.bit.QBP=0;	// Invert Pulse on QB

EQep1Regs.QEPCTL.bit.FREE_SOFT=2;
EQep1Regs.QEPCTL.bit.PCRM=0x00;	// QPOS reset on index.
EQep1Regs.QPOSMAX=	16384;		// 4096 pulses per rev * 4 from the decoder

EQep1Regs.QPOSCMP = 2048;		// Load the first compare value.
EQep1Regs.QPOSCTL.bit.PCE = 1;	// turn on position compare.
EQep1Regs.QCLR.all = 0xFFFF;	// clear all interrupts.
EQep1Regs.QEPCTL.bit.QPEN=1;	// QEP enable

	 	 return;
}


__interrupt void motor_isr(void) {

	if (sysFlags & laserState) {
		// Turn laser off
		GPIO_WritePin(laserPin,0);
		sysFlags &= ~laserState;
		EQep1Regs.QPOSCMP = laserPulse[0];
	} else {
		// turn laser on
		GPIO_WritePin(laserPin,1);
		sysFlags |= laserState;
		EQep1Regs.QPOSCMP = laserPulse[1];
	}
	EQep1Regs.QCLR.all = 0xFFFF;
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP5;

}

