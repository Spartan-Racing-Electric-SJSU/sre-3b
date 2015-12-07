/**************************************************************************
 * 	RINEHART HAND SHAKE PSEUDOCODE 
 *  ------------------------------
 *  REFERENCE: RMS CAN PROTOCOL - (PAGE 28)
 **************************************************************************
 *  This code will disable the lockout, enable the inverter, and command torque.
 *
 *  This feature is added so that the inverter cannot be accidentally enabled. 
 *	This feature requires that before sending out an Inverter Enable command, 
 *  the user must send out a Inverter Disable command. 
 *	Once the inverter sees a Disable command, 
 *  the lockout is removed, Rinehart can receive the Inverter Enable command,
 *  and torque can be set. 
 **************************************************************************
 *  TO-DO:
 *
 *	1.  Figure out why inverter isn't enabling;
 *		possible open circuit detection on 3 phase output?
 * 
 *  2.  Clean and organize code structure and logic; optimization
 *
 *  
 **************************************************************************
 * 	REVISION HISTORY: 
 
 *	2015-12-06 - Rabeel Elahi - Lockout disable command working
 *							  - Torque command working
 *							  - Added periodic timer for disable lockout message 
 *							  - CAN baud rate set to 500
 *							  
 *	2015-11-30 - Rabeel Elahi - Flashed code to VCU; started debugging code
 *							  -	The "create_CAN_frame" function is working
							  - The "hand_shake" function is working up to line 242
 *							  - Fixed compiler errors
 *							  - Fixed some code logic
 *						  
 *
 *	2015-11-25 - Rabeel Elahi - Created this file. 
 *              			  - Working on building and finalizing pseudocode. 
 *
 *
 *  
 *
 **************************************************************************/

#include "IO_CAN.h"
#include "IO_RTC.h" // RTC (REAL TIME CLOCK)
#include "APDB.h"



void hand_shake();
IO_CAN_DATA_FRAME create_CAN_frame(ubyte1 data_array[8], ubyte4 id);

void periodic_func();
IO_CAN_DATA_FRAME temp_frame;
IO_CAN_DATA_FRAME test_frame;

ubyte1 handle_w_PCAN, handle_r_PCAN, handle_w_RINEHART, handle_r_RINEHART;







APDB appl_db =
          { 0                      /* ubyte4 versionAPDB        */
          , {0}                    /* BL_T_DATE flashDate       */
                                   /* BL_T_DATE buildDate                   */
          , { (ubyte4) (((((ubyte4) 2015) & 0x0FFF) << 0) |
                        ((((ubyte4) "November") & 0x0F) << 12) |
                        ((((ubyte4) "Monday") & 0x1F) << 16) |
                        ((((ubyte4) 3) & 0x1F) << 21) |
                        ((((ubyte4) 17) & 0x3F) << 26)) }
          , 0                      /* ubyte4 nodeType           */
          , 0                      /* ubyte4 startAddress       */
          , 0                      /* ubyte4 codeSize           */
          , 0                      /* ubyte4 legacyAppCRC       */
          , 0                      /* ubyte4 appCRC             */
          , 1                      /* ubyte1 nodeNr             */
          , 0                      /* ubyte4 CRCInit            */
          , 0                      /* ubyte4 flags              */
          , 0                      /* ubyte4 hook1              */
          , 0                      /* ubyte4 hook2              */
          , 0                      /* ubyte4 hook3              */
          , APPL_START             /* ubyte4 mainAddress        */
          , {0, 1}                 /* BL_T_CAN_ID canDownloadID */
          , {0, 2}                 /* BL_T_CAN_ID canUploadID   */
          , 0                      /* ubyte4 legacyHeaderCRC    */
          , 0                      /* ubyte4 version            */
          , 500                    /* ubyte2 canBaudrate        */
          , 0                      /* ubyte1 canChannel         */
          , {0}                    /* ubyte1 reserved[8*4]      */
          , 0                      /* ubyte4 headerCRC          */
          };

		  
		
				
void main (void)
{
	
		IO_CAN_Init( IO_CAN_CHANNEL_1 // INITIALIZE CAN 1
			, 500
			, 0     //default
			, 0     //default
			, 0);   //default
			
		IO_CAN_Init( IO_CAN_CHANNEL_0 // INITIALIZE CAN 0
			, 500
			, 0     //default
			, 0     //default
			, 0);   //default

		IO_CAN_ConfigMsg( &handle_w_PCAN
				, IO_CAN_CHANNEL_0
				, IO_CAN_MSG_WRITE
				, IO_CAN_STD_FRAME
				, 0
				, 0);
				
		IO_CAN_ConfigMsg( &handle_r_PCAN
				, IO_CAN_CHANNEL_0
				, IO_CAN_MSG_WRITE
				, IO_CAN_STD_FRAME
				, 0
				, 0);
				
		IO_CAN_ConfigMsg( &handle_w_RINEHART
				, IO_CAN_CHANNEL_1
				, IO_CAN_MSG_WRITE
				, IO_CAN_STD_FRAME
				, 0
				, 0);

		IO_CAN_ConfigMsg( &handle_r_RINEHART
				, IO_CAN_CHANNEL_1
				, IO_CAN_MSG_READ
				, IO_CAN_STD_FRAME
				, 0
				, 0);
	
	ubyte4 time_stamp_0; // timestamp variables for different timing tasks
	IO_CAN_DATA_FRAME can_frame;
	IO_CAN_DATA_FRAME debug_frame; // Temporary CAN frame to store CAN messages
	IO_RTC_Init(); // Initialize RTC driver
	
	ubyte1 debug_array[8] = {1,2,3,4,5,6,7,8};
	ubyte4 debug_id = 0xFC;
	debug_frame = create_CAN_frame(debug_array, debug_id);
	
	
		
	while(1){

		// (RTC) REAL TIME CLOCK -- SEE IO_RTC.h in VCU API DOCUMENT //

		IO_RTC_StartTime(&time_stamp_0); // start time (get timestamp)

		
		
		
		//The function below returns the time in u-seconds which has passed
		//since the given timestamp has been taken via IO_RTC_StartTime()

		// FETCH LOCKOUT IS ENABLED MESSAGE (CAN ID: 0xAA) //
		while(IO_RTC_GetTimeUS(time_stamp_0) < 10000){ // checks to see if 5 ms have passed
		
			IO_CAN_ReadMsg(handle_r_RINEHART, &can_frame);
			
				if(can_frame.id == 0xAA && can_frame.data[0] == 0x04){
					break;
				}
				else{
					continue;
				}
		}
		
		// CHECK IF LOCKOUT IS ENABLED (BYTE 6 = 128); IF YES, INITIALIZE HAND_SHAKE SEQUENCE
		if(can_frame.id == 0xAA && can_frame.data[6] == 0x80 ){ 
		//IO_CAN_WriteMsg(handle_w_PCAN, &debug_frame);
			hand_shake();
		}
		else{
			continue;
		}

	}
}

	
	// FUNCTION DEFINITIONS // 
	
	void periodic_func(){
		
		IO_CAN_WriteMsg(handle_r_PCAN, &temp_frame);
		IO_CAN_WriteMsg(handle_w_RINEHART, &temp_frame);
		
	}

	void hand_shake(){

		int i = 0;
		ubyte1 array[8] = {0};
		ubyte4 id = 0;
		ubyte1 debug_array[8] = {1,2,3,4,5,6,7,8};
		ubyte4 debug_id = 0xFC;
		
		IO_CAN_DATA_FRAME debug_frame;
		debug_frame = create_CAN_frame(debug_array,debug_id);
		
		ubyte4 time_stamp_1;
		

		// CREATE LOCKOUT DISABLE MESSAGE //

		id = 0xC0; // CAN ID FOR DISABLE COMMAND
		for(i=0; i<8; i++){ // SET ALL BITS SET TO 0
			array[i] = 0;
		} 

		// PASS ID AND DATA ARRAY TO FUNCTION THAT WILL RETURN FULL CAN FRAME WITH PASSED VALUES
		temp_frame = create_CAN_frame(array, id);
		
		
	 // while(1){
				// IO_CAN_ReadMsg(handle_r_RINEHART, &test_frame);
			 // IO_CAN_WriteMsg(handle_w_PCAN, &test_frame);
	// }
		
		
		// WRITE MESSAGE TO RINEHART EVERY 60ms //
		
		IO_RTC_PeriodicInit(60000, &periodic_func);
		

		// VERIFY THAT LOCKOUT IS DISABLED //
			
			while(1){
			IO_CAN_ReadMsg(handle_r_RINEHART, &test_frame);
			if(test_frame.id == 0xAA){
				IO_CAN_WriteMsg(handle_w_PCAN, &test_frame);
				break;
			}
			else{
				continue;
			}
				}
		
		

		//IF LOCKOUT IS DISABLED, ENABLE INVERTER AND SET TORQUE TO +10 Nm
		if(test_frame.id == 0xAA){
		

			//CREATE MESSAGE TO ENABLE INVERTER AND SET TORQUE TO +10 Nm
			array[0] = 0x64; // TORQUE
			array[4] = 0x01;  // INVERTER ENABLE
			array[5] = 0x01;  // INVERTER ENABLE
			id = 0xC0;
			
			//PASS FRAME DATA (MESSAGE ID AND DATA ARRAY) TO CREATE CAN MESSAGE
			test_frame = create_CAN_frame(array, id);
			
			
			IO_RTC_PeriodicDeInit();
	
			
			// WRITE ENABLE INVERTER AND SET TORQUE TO +10 Nm MESSAGE TO CAN //
			IO_CAN_WriteMsg(handle_w_RINEHART, &test_frame);
					
  }
  
  // while(1){
				 // IO_CAN_ReadMsg(handle_r_RINEHART, &temp_frame);
			  // IO_CAN_WriteMsg(handle_w_PCAN, &temp_frame);
		// }
		return;
	}

	IO_CAN_DATA_FRAME create_CAN_frame(ubyte1 data_array[8], ubyte4 id){
		int i = 0;
		IO_CAN_DATA_FRAME temp_frame;


		//-----DEFAULT-----//

		temp_frame.id_format = IO_CAN_STD_FRAME;
		temp_frame.length = 8;

		//-----PASSED DATA-------//

		temp_frame.id = id;
		for(i=0; i<8; i++){
			temp_frame.data[i] = data_array[i];
		}

		return temp_frame;
	}