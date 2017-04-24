static const char Command_AT[]         = "AT\r";
static const char Command_RESET[]      = "AT+CFUN=1,1\r";                        /* soft reset */
static const char Command_CMGL[] 	   = "AT+CMGL=4\r";                          /* Read SMS*/
static const char Command_CPIN[]       = "AT+CPIN?\r";                           /* get pin status*/
static const char Command_CMER[] 	   = "AT+CMER=2,0,0,2,0\r";                  /* enable unsolicited message codes*/
static const char Command_CREG[]       = "AT+CREG?\r";
static const char Command_CGSN[]       = "AT+CGSN\r";                          	 /* Read IMEI*/
static const char Command_CIMI[] 	   = "AT+CIMI\r";
static const char RespOK[]             = "OK";
static const char RespReady[]          = "READY";
static const char RespCREG[]           = "+CREG: ";
static const char RespCSQ[]            = "+CSQ: ";
static const char RespCMGR[]           = "+CMGR:";
#ifdef T0_TELIT_GL865
static const char Command_SLED[]         = "AT#SLED=";
static const char Command_ATE0[]  	     = "ATE0;+CMEE=1;#SLED=3,3,7;+CREG=0;+CGREG=0;+CIND=0,0,0,0,0,0,0,0,0;+CNMI=2,1,0,0,0;+CMGF=1\r";
static const char Command_DefinePDP[]    = "AT+CGDCONT=1,\"IP\",\"";
static const char Command_CMGR[] 	     = "AT+CMGR=1;+CSQ;+CREG?;#CBC;+CMGD=1,4;#SI=1;#SS=1\r";
static const char Command_CMGS[]         = "AT+CMGS=";
static const char Command_SCFG[]         = "AT#SCFG=1,1,300,480,100,1\r";
static const char Command_SCFGEXT[]      = "AT#SCFGEXT=1,0,0,0,0,0\r";         	 /* socket configuration */
static const char Command_RecvData[]     = "AT#SRECV=1,256\r";
static const char Command_SendData[]     = "AT#SSENDEXT=1,";                  	 /* send data */
static const char Command_CloseSocket[]  = "AT#SH=1\r";
static const char Command_TCPConnect[]   = "AT#SD=1,0,";
static const char Command_SI[]    	     = "AT#SI=1\r";
static const char Command_ActivatePDP[]  = "AT#SGACT=1,1,\"";
static const char RespCBC[]              = "#CBC";
static const char RespSI[]               = "#SI:";
static const char RespSS[]               = "#SS:";
static const char RespSRECV[]            = "SRECV:";
#elif  defined(T0_QUECTEL_M66)
static const char Command_ATE0[]  	        = "ATE0;+CMEE=1;+QIURC=0;+CREG=0;+CGREG=0;+CMGF=1;+QSIMDET=0,0,0\r";
static const char Command_DefinePDP[]       = "AT+QIFGCNT=0\r";
static const char Command_CMGR[] 	        = "AT+CMGR=1;+CSQ;+CREG?;+CBC;+CMGD=1,4;+QISTAT;+QISACK\r";
static const char Command_CMGS[]            = "AT+CMGS=\"";
static const char Command_SendData[]        = "AT+QISEND=";
static const char Command_RecvData[]        = "AT+QIRD=1,1,0,";
static const char Command_SetRecvHdr[]      = "AT+QIHEAD=0;+QISHOWPT=0\r";         /* add TCP and IP headers */
static const char Command_CloseSocket[]     = "AT+QICLOSE\r";
static const char Command_SetIpAddrMode[]   = "AT+QIDNSIP=";
static const char Command_SetFgndContext[]  = "AT+QIFGCNT=";
static const char Command_SetRecvIndiMode[] = "AT+QINDI=";
static const char Command_TCPConnect[]      = "AT+QIOPEN=\"TCP\",\"";
static const char Command_ActivatePDP[]     = "AT+QICSGP=1,\"";
static const char RespConnectOK[]           = "CONNECT OK";
static const char RespPdpDeact[]            = "PDP DEACT";
static const char RespIpClose[]             = "IP CLOSE";
static const char RespAlrdyConnect[]        = "ALREADY CONNECT";
static const char RespSendOK[]              = "SEND OK";
static const char RespCBC[]                 = "+CBC";
static const char RespQISACK[]              = "+QISACK:";
#elif defined(T0_SIMCOM_SIM800C)
static const char Command_DefinePDP[]     = "AT+CGDCONT=1,\"IP\",\"";
static const char Command_CMGR[] 	      = "AT+CSQ;+CREG?\r";
static const char Command_SetRecvHdr[]    = "AT+CIPHEAD=1;+CIPSHOWT=1\r";         /* add TCP and IP headers */
static const char Command_ActivatePDP[]   = "AT+CSTT=\"";
static const char Command_ActivateGPRS[]  = "AT+CIICR\r";
static const char Command_TCPConnect[]    = "AT+CIPSTART=\"TCP\",\"";
static const char Command_SetRxMode[]     = "AT+CIPRXGET=1";
static const char Command_RecvData[]      = "AT+CIPRXGET=2,1";
static const char Command_SendData[]      = "AT+CIPSEND=";
static const char Command_CloseSocket[]   = "AT+QICLOSE=1\r";                     /* only use quick close */
static const char Command_GetLocalIP[]    = "AT+CIFSR\r";
static const char RespConnectOK[]         = "CONNECT OK";
static const char RespIpClose[]           = "IP CLOSE"
static const char RespPdpDeact[]          = "PDP DEACT"
static const char RespCBC[]               = "+CBC";
static const char RespQISACK[]            = "+QISACK:";
static const char RespSendOK[]            = "SEND OK";
static const char Command_CMGS[]         = "AT+CMGS=";
static const char Command_ATE0[]  	     = "ATE0;+CMEE=1;#SLED=2;+CREG=0;+CGREG=0;+CIND=0,0,0,0,0,0,0,0,0;+CNMI=2,1,0,0,0;+CMGF=1\r";
static const char RespAlrdyConnect[]        = "ALREADY CONNECT";

#endif

                    		 /* close socket connection*/
//static const char Command_SD[]         = "AT#SD=1,0,10510,\"modules.telit.com\",0,0,1";  /* connect to telit echo server*/
//static const char Command_SD[]         = "AT#SD=1,0,6081,\"178.63.30.81\",0,0,1\r";  /* connect to trio server */
//static const char Command_SD[]         = "AT#SD=1,0,1555,\"213.14.184.87\",0,0,1\r";  /* connect to my computer */



#define DATA_SEND_PROMPT   '>'
#define CHAR_CR            '\r'
#define CHAR_LF            '\n'
#define CHAR_ENTER         "\r\n"
#define CTRL_Z              0x1A
