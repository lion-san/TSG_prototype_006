//------------------------------------------------------------
//    姿勢制御フィルタリングプログラム
//                Arduino　IDE　1.6.11
//
//　　　Arduino　　　　　　　　LSM9DS1基板　
//　　　　3.3V　　　------　　　　3.3V
//　　　　GND       ------   　　 GND
//　　　　SCL       ------        SCL
//　　　　SDA       ------        SDA
//
//　GPSで日時取得しファイル名とする
//  電源ONで記録開始
//  その後9軸センサーデータを記録する
//
//　　　　
//----------------------------------------------------------//


#include <SPI.h>                                //SPIライブラリ
#include <Wire.h>                               //I2Cライブラリ
#include <SparkFunLSM9DS1.h>                  //LSM9DS1ライブラリ：https://github.com/sparkfun/LSM9DS1_Breakout
#include <SD.h>
#include <LSM9DS1_Registers.h>
#include <LSM9DS1_Types.h>
#include <SoftwareSerial.h>

//#define ADAddr 0x48//

#define LSM9DS1_M  0x1E                 // SPIアドレス設定 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B                // SPIアドレス設定 if SDO_AG is LOW

//#define PRINT_CALCULATED              //表示用の定義
//#define DEBUG_GYRO                    //ジャイロスコープの表示


#define RX 8                            //GPS用のソフトウェアシリアル
#define TX 9                            //GPS用のソフトウェアシリアル
#define SENTENCES_BUFLEN      128        // GPSのメッセージデータバッファの個数

//=== Global for GPS ===========================================
SoftwareSerial  g_gps( RX, TX );
char head[] = "$GPRMC";
char buf[10];
int SentencesNum = 0;                   // GPSのセンテンス文字列個数
byte SentencesData[SENTENCES_BUFLEN] ;  // GPSのセンテンスデータバッファ
char datetime_org[6];
String datetime = "";
String date = "";
boolean is_getdate = false;
//======================================================

//-------------------------------------------------------------------------
//[Global valiables]

LSM9DS1 imu;


//###############################################
//MicroSD 
//const int chipSelect = 4;//Arduino UNO
const int chipSelect = 10;//Arduino Micro

File dataFile;                          //SD CARD
boolean sdOpened = false;
//###############################################


char fileName[16];

volatile boolean enableWrite = false;

volatile unsigned long time;

////////////////////////////////////////////////////////////////




void setup(void) {

  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  g_gps.begin(9600);

  //=== SD Card Initialize ====================================
  Serial.print(F("Initializing SD card..."));
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println(F("Card failed, or not present"));
    // don't do anything more:
    return;
  }
  Serial.println(F("card initialized."));
  //=======================================================

  //LED
  pinMode(13, OUTPUT);
  
  //=== LSM9DS1 Initialize =====================================
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress  = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;

  if (!imu.begin())              //センサ接続エラー時の表示
  {
    Serial.println(F("Failed to communicate with LSM9DS1."));
    while (1)
      ;
  }
  //=======================================================

  //GPSより日付日時の取得
  getGpsData();

  //日付取得完了したら、クローズ
  g_gps.end();

  //SDカードのオープンとファイル名の取得
  sdcardOpen();

}



/**
 * 　==================================================================================================================================================
 * loop
 * ずっと繰り返される関数（何秒周期？）
 * 　==================================================================================================================================================
 */
void loop(void) {


  //SDカードへの出力
  writeDataToSdcard();

  digitalWrite(13, 1);


}


//==================================================================================================================================================


/**
 * sdcardOpen
 */
void sdcardOpen()
{
  // ファイル名決定
  String s;
  int fileNum = 0;
  
  while(1){
    s = datetime;
    /*if (fileNum < 10) {
      s += "00";
    } else if(fileNum < 100) {
      s += "0";
    }
    s += fileNum;*/
    s += ".csv";
    s.toCharArray(fileName, 16);
    if(!SD.exists(fileName)) break;
    fileNum++;
  }

}


/**
 * writeDataToSdcard
 */
 
void writeDataToSdcard()
{

  updateMotionSensors();
          

  //file open
  dataFile = SD.open(fileName, FILE_WRITE);


  // if the file is available, write to it:
  if (dataFile) {

    sdOpened = true;


    //===記録======================
    //時間の更新
    dataFile.print(millis() - time);
    time = millis();
    dataFile.print(",");


    //[g]
    dataFile.print(imu.calcAccel(imu.ax));
    dataFile.print(",");
    dataFile.print(imu.calcAccel(imu.ay));
    dataFile.print(",");
    dataFile.print(imu.calcAccel(imu.az));
    dataFile.print(",");

    //[deg/s]
    dataFile.print(imu.calcGyro(imu.gx));
    dataFile.print(",");
    dataFile.print(imu.calcGyro(imu.gy));
    dataFile.print(",");
    dataFile.print(imu.calcGyro(imu.gz));
    dataFile.print(",");

    //[gauss]
    dataFile.print(imu.calcMag(imu.mx));
    dataFile.print(",");
    dataFile.print(imu.calcMag(imu.my));
    dataFile.print(",");
    dataFile.print(imu.calcMag(imu.mz));
  
  
    dataFile.print("\n");
    //=============================
    dataFile.close();
    

    //file close
    dataFile.close();
    //sdOpened = false;

  }
  // if the file isn't open, pop up an error:
  else {
     Serial.println("fileError");
  }

}


/**
 * updateMotionSensors
 * 
 */

void updateMotionSensors()
{
  imu.readGyro();
  imu.readAccel();
  imu.readMag();    
}


/**
 * 　==================================================================================================================================================
 * getGpsData
 * GPSチップより、情報を取得する
 * 　==================================================================================================================================================
 */
void getGpsData(void) {

  while(1){
      //GPS MAIN ==========================================================
      char c = 0 ;
    
        // センテンスデータが有るなら処理を行う
        if (g_gps.available()) {
    
            // 1バイト読み出す
            c = g_gps.read() ;
            Serial.write(c);//Debug ALL
    
            // センテンスの開始
            if (c == '$') SentencesNum = 0 ;
            
            if (SentencesNum >= 0) {
              
              // センテンスをバッファに溜める
              SentencesData[SentencesNum] = c ;
              SentencesNum++ ;
                 
              // センテンスの最後(LF=0x0Aで判断)
              if (c == 0x0a || SentencesNum >= SENTENCES_BUFLEN) {
        
                SentencesData[SentencesNum] = '\0';
  
                Serial.print( (char *)SentencesData );
  
                //GPS情報の取得
                getGpsInfo();

                if(is_getdate){
                  return;
                }
              }
            }
          }
      //GPS MAIN ==========================================================
  }

}

/**
 * getGpsInfo
 * $GPGGA　ヘッダから、衛星受信数や時刻情報を取得
 */
void getGpsInfo()
{
    int i, c;
    
    //$1ヘッダが一致
    if( strncmp((char *)SentencesData, head, 6) == 0 )
    {

      //コンマカウント初期化
      c = 1; 

      // センテンスの長さだけ繰り返す
      for (i=0 ; i<SentencesNum; i++) {
        if (SentencesData[i] == ','){
          
            c++ ; // 区切り文字を数える
    
            if ( c == 2 ) {;
                 strncpy(datetime_org, readDataUntilComma(i+1), 6);
                 datetime = datetime_org;
                 Serial.println(datetime);
                 continue;
            }
            else if ( c == 10 ) {
                 date = readDataUntilComma(i+1);
                 Serial.println(date);
                 is_getdate = true;
                 return;
            }
        }
      }
    }
}
/**
  * readDataUntilComma
  */
char* readDataUntilComma(int s)
{
  int i, j;

  j = 0;
  //初期化
  memset(buf,0x00,sizeof(buf)) ;

  //終了条件
  //次のコンマが出現or特定文字*（チェックサム)が出現
  for (i = s; i < SentencesNum; i++)
  {
    if(( SentencesData[i] == ',') || (SentencesData[i] == '*')){
      buf[j] = '\0';
      return buf;
    }
    else{
      //バッファーのオーバフローをチェック
      if( j < sizeof(buf) ) {
        buf[j] = SentencesData[i];
        j++;
      }
      else{//エラー処理
        int x;
        for(x = 0; x < sizeof(buf); x++)
          buf[x] = 'X';
          return buf;
      }
      
    }
  }
  
}

