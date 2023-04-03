
/**
  OC3 Generated Driver API Source File

  @Company
    Microchip Technology Inc.

  @File Name
    oc3.c

  @Summary
    This is the generated source file for the OC3 driver using PIC24 / dsPIC33 / PIC32MM MCUs

  @Description
    This source file provides APIs for driver for OC3.
    Generation Information :
        Product Revision  :  PIC24 / dsPIC33 / PIC32MM MCUs - 1.171.1
        Device            :  PIC24FJ64GB202
    The generated drivers are tested against the following:
        Compiler          :  XC16 v1.70
        MPLAB 	          :  MPLAB X v5.50
*/

/*
    (c) 2020 Microchip Technology Inc. and its subsidiaries. You may use this
    software and any derivatives exclusively with Microchip products.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
    WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
*/

/**
  Section: Included Files
*/

#include "oc3.h"

/** OC Mode.

  @Summary
    Defines the OC Mode.

  @Description
    This data type defines the OC Mode of operation.

*/

static uint16_t         gOC3Mode;

/**
  Section: Driver Interface
*/


void OC3_Initialize (void)
{
    // ENFLT0 disabled; ENFLT1 disabled; OCFLT2 disabled; ENFLT2 disabled; OCSIDL disabled; OCM Edge-Aligned PWM mode; OCFLT1 disabled; OCFLT0 disabled; OCTSEL FOSC/2; TRIGMODE Only Software; 
    OC3CON1 = 0x1C06;
    // SYNCSEL Self; TRIGSTAT disabled; DCB Start of instruction cycle; OCINV disabled; OCTRIG Sync; OC32 disabled; FLTOUT disabled; OCTRIS disabled; FLTMD Cycle; FLTTRIEN disabled; 
    OC3CON2 = 0x1F;
    // OC3RS 255; 
    OC3RS = 0xFF;
    // OC3R 128; 
    OC3R = 0x80;
	
    gOC3Mode = OC3CON1bits.OCM;
}

void __attribute__ ((weak)) OC3_CallBack(void)
{
    // Add your custom callback code here
}

void OC3_Tasks( void )
{	
    if(IFS1bits.OC3IF)
    {
		// OC3 callback function 
		OC3_CallBack();
        IFS1bits.OC3IF = 0;
    }
}



void OC3_Start( void )
{
    OC3CON1bits.OCM = gOC3Mode;
}


void OC3_Stop( void )
{
    OC3CON1bits.OCM = 0;
}

void OC3_SecondaryValueSet( uint16_t secVal )
{
   
    OC3RS = secVal;
}


void OC3_PrimaryValueSet( uint16_t priVal )
{
   
    OC3R = priVal;
}

bool OC3_IsCompareCycleComplete( void )
{
    return(IFS1bits.OC3IF);
}


bool OC3_FaultStatusGet( OC3_FAULTS faultNum )
{
    bool status;
    /* Return the status of the fault condition */
   
    switch(faultNum)
    { 
        case OC3_FAULT0:
			status = OC3CON1bits.OCFLT0;
            break;
        case OC3_FAULT1:
			status = OC3CON1bits.OCFLT1;
            break;
        case OC3_FAULT2:
			status = OC3CON1bits.OCFLT2;
            break;
        default :
            break;

    }
    return(status);
}


void OC3_FaultStatusClear( OC3_FAULTS faultNum )
{
    
    switch(faultNum)
    { 
        case OC3_FAULT0:
			OC3CON1bits.OCFLT0 = 0;
                break;
        case OC3_FAULT1:
			OC3CON1bits.OCFLT1 = 0;
                break;
        case OC3_FAULT2:
			OC3CON1bits.OCFLT2 = 0;
                break;
        default :
                break;
    }    
}


void OC3_ManualTriggerSet( void )
{
    OC3CON2bits.TRIGSTAT= true; 
}

bool OC3_TriggerStatusGet( void )
{
    return( OC3CON2bits.TRIGSTAT );
}


void OC3_TriggerStatusClear( void )
{
    /* Clears the trigger status */
    OC3CON2bits.TRIGSTAT = 0;
}

/**
 End of File
*/
