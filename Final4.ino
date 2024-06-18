//ちょっとアイディア出しをしたい//
//どう足掻いてもblueとredのゴールが安定してくれない。//

#include <Wire.h>

#define fanpin 2
#define lwheel 11
#define rwheel 6
#define lwheel_a 7
#define rwheel_a 10
#define step0 0
#define step1 1
#define step2 2
#define stepB 3
#define stepY 4
#define stepR 5
//rとlはright,leftの意味。それぞれの頭について左か右かを表す//
//wheelをHIGH、LOWと切り替えることでステッピングモーターが回転する//
//wheel_aはそれぞれの車輪の回転方向を決定する。。aはアングルの意味//
//stepは、左右の車輪を制御するときの状態を表すもの。step1ならHIGHにする。step2ならLOWにする。//


//6月9日、wheelのdefineを左右逆にした//
//6月12日13時ごろ、bgoal→crossreadのテストが完了。なんか怖かったんでFinal3にバックアップしました。//
//6月13日、angleを逆にした。//

#define rsensor_o A0
#define rsensor_i A2
#define lsensor_o A1
#define lsensor_i A3
//o,iはそれぞれout(外側)、in（内側）の略//

#define lightpin A7   //センサ信号入力
#define Vcc (float)5  //電源電圧

#define I2C_SDA A4
#define I2C_SCL A5
#define S11059_ADDR 0x2A


int straight(int straightspeed, int straighttimes);
int back(int backspeed, int backtimes);
int left(int lspan);
int right(int rspan);
int rcondition = step1;
int lcondition = step1;
unsigned long last_rtime = 0;
unsigned long last_ltime = 0;
//spanは、HIGH,LOW切り替えのスパン//
//conditionは現在の状態を記録する//
//last_timeは、最後に右、左のモーターのHIGH,LOWを切り替えた時刻を記録する//

int rturn(int rturnspeed, int rangle);
int lturn(int lturnspeed, int langle);
int rturn90(int rturnspeed, int rquater);
int lturn90(int lturnspeed, int lquater);
int rturnsense(int rturnspeed, int rangle);
int linewheelcount = 0;
int wheeltimes = 0;
int ywheeltimes = 0;
int rwheeltimes = 0;
int rttimes = 0;
int lttimes = 0;
int rt90times = 0;
int lt90times = 0;
  int last_hosei = 0;
//rturn(回転速度,rightの実行回数)で使用する。rightの実行回数とあるが、厳密には、「HIGHとLOWの切り替え回数/2」。rightは2回実行して１往復なのに対し、rturn内部の処理では1回実行して２往復であるため//
//rturn90(回転速度,90°回る回数)で使用する//
//wheeltimesは、何回rightturn系統の関数内でHIGHLOWの切り替えを実行したかを記録する//
//rt90timesは、rturn90そのものを、loop全体において何回実行したかを記録し続ける。lt90times, rttimesについても同様。多分本番では使わないやつ//

int bgoal(int goalspeed);
//bluegoalの略。直進し、黒線を検出したら停止し、ファンを停止させて、逆向きに同じ数だけ進み、180°回転する//
//180°回転の精度がクソ//

int readphoto(int photospan);
int rdata_o = 0;
int rdata_i = 0;
int ldata_o = 0;
int ldata_i = 0;
unsigned long last_phototime = 0;
//data_oやdata_iは、フォトリフレクターが読み取った値を記録する//
//photospanはフォトリフレクタ－を読み取る周期//
//last_phototimeは、最後にフォトリフレクターの値を読み取った時刻を記録する//

int pid(int *p_photospan);
float diff = 0;
float last_diff = 0;
float sum_diff = 0;
float derivative;
int hosei = 0;
//pid関数はpid制御の補正の値を求める。返り値は補正値//
//diffは、左右のフォトリフレクターの値の差。ラインより右側なら正の値//
//last_diffはd制御のために使います//
//derivativeは微分の値です//
//sum_diffは、i制御のために使います。目標との差の合計です//

int linetrace(int speed, int crossnumber_ex);
int linetrace2(int speed, int linewheeltimes);
//ライントレースします。crossnumber_exの数だけ十字を見つけたら、停止します//

unsigned long last_crosstime1 = 0;
unsigned long last_crosstime2 = 0;
int crossnumber = 0;
//これまで通過した十字の数を記録する//

float distance_1;
float distance_2;
int readlight(void);
//distance_2が実際の距離（cm）//


int goandback(int gbspeed);
//gbspeedは、go and back speedの略//

int rcheckcrossb(int checkspeed);
int rcheckcrossy(int checkspeed);


void readcolor(void);
char color;
//colorは、カラーセンサー起動時、自分が持っている球の色を記録する。それぞれの色の頭文字（R,B,Y）あるいは何も持っていない（N）//


int sensegoback(int rturnspeed, int rangle, int gbspeed);
//球がどこにあるのかを特定し、ファンを起動。直進したのちに、元の場所まで戻ってくるプログラム//


int ball = 0;
int phase = step0;


void setup() {



  pinMode(lwheel_a, OUTPUT);
  pinMode(lwheel, OUTPUT);
  pinMode(rwheel_a, OUTPUT);
  pinMode(rwheel, OUTPUT);
  pinMode(fanpin, OUTPUT);

  delay(100);

  digitalWrite(rwheel_a, LOW);
  digitalWrite(lwheel_a, HIGH);
  //モーターの回転方向の決定。これで前進する//

  #if 1
  Serial.begin(9600);
  Serial.println("START");  //このSTARTは、デバックに便利でした//
  delay(100);

  Wire.begin(I2C_SDA);
  Wire.begin(I2C_SDA);
  Wire.begin(S11059_ADDR);  //初期化を行います。何のために初期化しているのか、果たしてこれが初期化なのか、私にはわかりません//

  Wire.beginTransmission(S11059_ADDR);
  Wire.write(0x00);
  Wire.write(0x84);
  Wire.endTransmission();  //初期化の一環だと思います//

  delay(100);

  Wire.beginTransmission(S11059_ADDR);
  Wire.write(0x00);
  Wire.write(0x0A);  //ここで積分時間とゲインを決定しています。読み取った値が大きすぎる・小さすぎる・そもそも変化しない場合は、たいていここを治せばなんとかなります//
  Wire.endTransmission(S11059_ADDR);

  delay(100);
  //積分時間（今回は多分22.4ms）より長く待機しなくてはならないそうです//
  #endif
  
}



//全体の状況はphaseで管理される//
//step0はスタート直後～ボール収集//
//step1は、ボール収集//
//stepB,R,Yは、それぞれのゴール～ボール収集エリアへの移動//
void loop() {

  #if 1 //始　これはゲームが始まってから、ボールを探知する位置まで行くためのプログラム。全くテストしてません。特にreadcrossの周期がこれでいいのか怖いです//

    if(phase == step0){
    crossnumber=0;
    for(;;)
   {

    right(3);
    left(3);
    readcross(5,700);

    if(crossnumber == 2)
      {

      straight(5,300);
      Serial.println("LINETRACE IS NOW STARTED");
      crossnumber=0;
      linetrace(5,3);
      phase  = step1;
      break;
      }
    }
    }

  #endif
  //まっすぐ進み、通過した横棒の数が2になったら、追加で少し進んだのち、ライントレースに切り替える//
  //ライントレースは、ボール収集可能な地点まで続く//



  #if 1 //探　これはボールを探し当てるプログラム。for文による永遠ループを実装した。入手したボール数による処理と、ボールが見つからないときの処理を追加した。確認すること//

    if(phase ==  step1){
    for(;;)
    {
      back(5, 30);

      if(ball >= 2)
      {
        straight(5, 150);
      }


      if(ball >= 4)
      {
        straight(5, 150);
      }


      if(ball >= 6)
      {
        straight(5, 150);
      }

      if(ball >= 8)
      {
        straight(5, 150);
      }

      if(ball >= 10)
      {
        straight(5, 150);
      }


      sensegoback(20, 200, 6);
      readcolor();
      linetrace2(5, 120);

      if(color == 'R')
      {
        phase = stepR;
        break;
      }    
  
      else if(color == 'B')
      {
        phase = stepB;
        break;
      }

      else if(color == 'Y')
      {
        phase = stepY;
        break;
      }

      back(5, 120);
    }

    back(5, 120);
      if(ball >= 2)
      {
        back(5, 150);
      }


      if(ball >= 4)
      {
        back(5, 150);
      }


      if(ball >= 6)
      {
        back(5, 160);
      }

      if(ball >= 8)
      {
        back(5, 150);
      }

      if(ball >= 10)
      {
        back(5, 150);
      }





    lturn(10,160);
    for(;;)
    {
      lturn(10, 15);
      readphoto(10);
      if((ldata_i) < 50)
      {
        break;
      }
    }
    }
  #endif
  //ボール数が多ければ余分に前進する。この場合、角度調整した後、調整のために余分に後退する。テストしてないです//



  #if 1 //赤　赤いボールをゴールに入れるプログラムその２！　６月１４日追記：完成したぞ……//
  
    if(phase == stepR){
    linetrace(5,3);
    ygoal2(10);
    crossnumber = 0;
    linetrace(5,3);
    phase = step1;
    }
  #endif



  #if 1  //黄　ここは、黄色いボールをゴールに入れる用のプログラムその２！！　いれたらボール収集まで戻ってくる。多分こっちの方が安定する。6月14日追記：完成したぞ……//  

    if(phase == stepY){
    linetrace(5,2);
    crossnumber = 0;
    ygoal2(10);
    linetrace(5,2);
    phase = step1;
    Serial.println(phase);
    }
  #endif



  #if 1  //青　ここは、青いボールを入れる用のプログラム！　いれたら、ボール収集エリアまで戻ってくる//

    if(phase == stepB){
    linetrace(5,1);
    bgoal2(7);
    crossnumber = 0;
    linetrace(5, 2);
    phase = step1;
    }
  #endif




  #if 0  //本番で使用しないプログラム達//

    if(phase == stepY){//黄色いボールをゴールに入れるプログラム//
    linetrace(5,1);
    ygoal(10);
    crossnumber = 0;
    linetrace(5,2);
    phase = step1;
    }


    
    if(phase == stepR){//赤いボールをゴールに入れるプログラム。最後はライントレースでごり押してる。なんて原始的なんだ……//
    linetrace(5,1);
    rgoal(10);
    crossnumber = 0;
    linetrace(5,3);
    phase = step1;
    }
  #endif

}





int straight(int straightspeed, int straighttimes) {
  int i;

  for (i = 0; i < straighttimes; i++) {
    digitalWrite(lwheel, HIGH);
    digitalWrite(rwheel, HIGH);
    delay(straightspeed);
    digitalWrite(lwheel, LOW);
    digitalWrite(rwheel, LOW);
    delay(straightspeed);
  }
}


int back(int backspeed, int backtimes) {
  int i;
  digitalWrite(rwheel_a, HIGH);
  digitalWrite(lwheel_a, LOW);

  for (i = 0; i < backtimes; i++) {

    digitalWrite(lwheel, HIGH);
    digitalWrite(rwheel, HIGH);
    delay(backspeed);
    digitalWrite(lwheel, LOW);
    digitalWrite(rwheel, LOW);
    delay(backspeed);
  }

  digitalWrite(rwheel_a, LOW);
  digitalWrite(lwheel_a, HIGH);
}


int right(int rspan) {
  if (millis() - last_rtime > rspan) {
    switch (rcondition) {
      case step1:
        digitalWrite(rwheel, HIGH);
        rcondition = step2;
        break;

      case step2:
        digitalWrite(rwheel, LOW);
        rcondition = step1;
        ywheeltimes++;
        rwheeltimes++; //これらはrcheckcrossy,rcheckcrossrなどで使用。rightとleftの往復回数を記録する//
        linewheelcount++;
        break;
    }
    last_rtime = millis();
  }
}
//最後にHIGH,LOWを切り替えた時刻がspan以上なら、モーターのHIGH,LOWを切り替える関数。HIGHならLOWに、LOWならHIGHに切り替える//


int left(int lspan) {
  if (millis() - last_ltime > lspan) {
    switch (lcondition) {
      case step1:
        digitalWrite(lwheel, HIGH);
        lcondition = step2;
        break;

      case step2:
        digitalWrite(lwheel, LOW);
        lcondition = step1;
        break;
    }
    last_ltime = millis();
  }
}


int rturn(int rturnspeed, int rangle) {
  digitalWrite(rwheel_a, HIGH);
  digitalWrite(lwheel_a, HIGH);
  //右の車輪の回転向きを逆に、左の車輪はそのままにする。右回転する準備//

  for (wheeltimes = 0; wheeltimes <= (double)(rangle) / 2; wheeltimes++) {
    digitalWrite(lwheel, HIGH);
    digitalWrite(rwheel, HIGH);
    delay(rturnspeed);
    digitalWrite(lwheel, LOW);
    digitalWrite(rwheel, LOW);
    delay(rturnspeed);
    //指定回数（＝rangle）/2回だけ、HIGHLOWを往復する⇔指定回数だけHIGHLOWの切り替えを行う//
  }
  digitalWrite(rwheel_a, LOW);
  digitalWrite(lwheel_a, HIGH);
  //車輪回転の向きを基に戻す//

  rttimes++;

  //rttimesは、rttimesを実行した回数を記録する。具体的には、loop中でrttimesが複数回実行されないようにするための状況確認用のに用いる//
}


int lturn(int lturnspeed, int langle) {
  digitalWrite(rwheel_a, LOW);
  digitalWrite(lwheel_a, LOW);

  for (wheeltimes = 0; wheeltimes <= (double)(langle) / 2; wheeltimes++) {
    digitalWrite(lwheel, HIGH);
    digitalWrite(rwheel, HIGH);
    delay(lturnspeed);
    digitalWrite(lwheel, LOW);
    digitalWrite(rwheel, LOW);
    delay(lturnspeed);
  }
  digitalWrite(rwheel_a, LOW);
  digitalWrite(lwheel_a, HIGH);
  lttimes++;
}
//仕組みはrturnと同じ//


int rturn90(int rturnspeed, int rquater) {
  digitalWrite(rwheel_a, HIGH);
  digitalWrite(lwheel_a, HIGH);

  for (wheeltimes = 0; wheeltimes <= (104 * rquater); wheeltimes++) {
    digitalWrite(lwheel, HIGH);
    digitalWrite(rwheel, HIGH);
    delay(rturnspeed);
    digitalWrite(lwheel, LOW);
    digitalWrite(rwheel, LOW);
    delay(rturnspeed);
  }
  digitalWrite(rwheel_a, LOW);
  digitalWrite(lwheel_a, HIGH);
  rt90times++;
}
//rturnとおおよそ同じ仕組み。ただし、90°になるように調整している//


int lturn90(int lturnspeed, int lquater) {
  digitalWrite(rwheel_a, LOW);
  digitalWrite(lwheel_a, LOW);

  for (wheeltimes = 0; wheeltimes <= (107 * lquater); wheeltimes++) {
    digitalWrite(lwheel, HIGH);
    digitalWrite(rwheel, HIGH);
    delay(lturnspeed);
    digitalWrite(lwheel, LOW);
    digitalWrite(rwheel, LOW);
    delay(lturnspeed);
  }
  digitalWrite(rwheel_a, LOW);
  digitalWrite(lwheel_a, HIGH);
  lt90times++;
}
//rturn90と同じ仕組み//


int rturnsense(int rturnspeed, int rangle) {
  float distance_2_ex[rangle];
  float distance_2_min;
  int distance_2_exn = 0;
  int wheeltimes_ex[rangle];
  int wheeltimes_min;
  int i;

  digitalWrite(rwheel_a, HIGH);
  digitalWrite(lwheel_a, HIGH);

  for (wheeltimes = 0; wheeltimes <= (float)(rangle) / 2; wheeltimes++) {
    digitalWrite(lwheel, HIGH);
    digitalWrite(rwheel, HIGH);
    delay(rturnspeed);
    digitalWrite(lwheel, LOW);
    digitalWrite(rwheel, LOW);
    delay(rturnspeed);
    //rturnと同じ。指定回数/2回だけHIGHLOWを往復する⇔指定回数だけHIGHLOWの切り替えを行う//

    readlight();
    //距離センサーを読み取り、値をdistance_2に格納//

    distance_2_exn++;
    distance_2_ex[distance_2_exn] = distance_2;
    wheeltimes_ex[distance_2_exn] = wheeltimes;
    Serial.print("distance_2_ex[");
    Serial.print(distance_2_exn);
    Serial.print("]");
    Serial.print("=");
    Serial.println(distance_2_ex[distance_2_exn]);
    //distance_2_ex[1], distance_2_ex[2], distance_2_ex[3]......に、distance_2を格納していく//
    //同様に、wheeltimes_exに、何回HIGHLOWの往復を行ったのかを記録する//
  }

  distance_2_min = distance_2_ex[1];
  //仮に、記録していたdistance_2_ex[]中の最小値を、distance_2_ex[1]とする//

  for (i = 1; i <= distance_2_exn; i++) {
    if (distance_2_ex[i] < distance_2_min) {
      distance_2_min = distance_2_ex[i];
      wheeltimes_min = wheeltimes_ex[i];
      Serial.println(distance_2_min);
    }
  }
  //distance_2_ex[]の最小値を、distance_2_minに記録する//
  //wheeltimes_minに、distance_2_minに至るまで何回HIGHLOWの往復を行ったのかを記録する//

  Serial.println(wheeltimes_min);
  lturn(10, rangle  -  wheeltimes_min * 2 + 25);
  //rangle/2は、HIGHLOWの往復回数（∵rangleがHIGHLOWの切り替え回数）。wheeltimes_minは、distance_2_minに至るまでのHIGHLOWの往復回数//
  //「HIGHLOWの全体の往復回数」ー「最小値に至るまでのHIGHLOWの往復回数」により、何回HIGHLOWを逆向きに往復させればよいのかを求める//
  //lturn(lturnspeed, langle)の、langleには、切り替え回数を代入しなくてはならない。よって２倍する//
  //ファンの位置と、センサーの位置にはズレがあるので13を引いて調整する//
  //距離センサーが右側についている⇔左に回る回数は理論値よりも少なくていい//



  //6月12日、このlturnを調整した。大丈夫？？？　動くよね？？？？//

  digitalWrite(rwheel_a, LOW);
  digitalWrite(lwheel_a, HIGH);
  rttimes++;
  return wheeltimes_min;
}
//右側に向かってゆっくりと回り、ボールがある場所を探知するプログラム//


int goandback(int gbspeed) {
  int gotimes = 0;
  //gotimesは、自分がどれだけ前進したか（HIGHLOWの往復を何回行ったか）を記録する//
  digitalWrite(fanpin, HIGH);

  for (;;) {
    digitalWrite(lwheel, HIGH);
    digitalWrite(rwheel, HIGH);
    delay(gbspeed);
    digitalWrite(lwheel, LOW);
    digitalWrite(rwheel, LOW);
    delay(gbspeed);
    gotimes++;

    if (gotimes % 10 == 0) {
      readcolor();
      if (!(color == 'N')) {
        break;
      }
    }


    //以下テスト。角度がズレたときのための応急処置//
    if(gotimes > 400)
    {
      break;
    }
    //HIGHLOWの繰り返しを50回行うたびに、カラーセンサーを読み取る//
    //カラーセンサーの読みとりそのものに時間がかかるため、HIGHLOWを切り替えるたびにreadcolorを実行すると車輪の回り方がおかしくなる//
    //カラーセンサーに色が検出された場合、車輪を回すのをやめる//
  }

  Serial.println("BROKEN");
  digitalWrite(rwheel_a, HIGH);
  digitalWrite(lwheel_a, LOW);
  //車輪を逆回転の状態にセットする//

  int i;

  for (i = 0; i < gotimes; i++) {
    digitalWrite(lwheel, HIGH);
    digitalWrite(rwheel, HIGH);
    delay(gbspeed);
    digitalWrite(lwheel, LOW);
    digitalWrite(rwheel, LOW);
    delay(gbspeed);
  }
  //来た時と同じ回数だけ、車輪を逆回転させる//

  digitalWrite(rwheel_a, LOW);
  digitalWrite(lwheel_a, HIGH);
  //車輪の状態をもとに戻す//
}
//前進し、もしカラーセンサーに反応があれば、元いた場所まで戻ってくる//


int sensegoback(int rturnspeed, int rangle, int gbspeed) {
  int returnangle;
  lturn(10, rangle/2);
  int wheeltimes_min = rturnsense(rturnspeed, rangle);
  //ボールの位置を探知し、その方向を向く。10ms間隔で、200回HIGHLOWを往復する。//

  goandback(gbspeed);
  //ファンを起動してその方向に直進し、元の場所まで戻ってくる//

  returnangle = rangle/2 - wheeltimes_min*2 + 25;
  //langle - rangle + (rangle - wheeltimes_min*2 - 13) - returangle = 0(左回転を正としている。returangleは右回転想定)//
  //⇔rangle/2 - rangle + rangle -wheeltimes_min*2 - 13 = returnangle//
  //returnangle =  langle - rangle + (langle -wheeltimes_min*2) (←？？？？？？)//
  //returnangleは、前向きに戻るために必要な角度（→回転を想定）//


  if(returnangle > 0)
  {
  rturn(10, returnangle);
  }
  //元に戻るのに必要な分だけ右回転//

  if(returnangle < 0)
  {
  lturn(10, -returnangle);
  }
  //必要な回転が左だったなら、左回転を行う//

}
//ボールを探知→前進→カラーセンサーに反応があればもといた場所まで戻る//
//6/12、角度ももとの場所まで戻る機能を実装した//


int linetrace(int speed, int crossnumber_ex) {

  crossnumber = 0;
  for (;;) {
    readphoto(20);
    if (hosei >= 0) {
      right(speed);
      left(speed + hosei);
    }

    if (hosei < 0) {
      right(speed - hosei);
      left(speed);
    }

  readcross(100, 1500);
    if (crossnumber == crossnumber_ex) {
      Serial.println("LINETRACE IS NOW BROKEN");
      break;
    }
  }
}
//指定したスピードでライントレースする。crossnumber_exの数だけ十字を通過すると、その場で停止する//

int linetrace2(int speed, int linewheeltimes)
  {
  for(linewheelcount = 0; linewheelcount < linewheeltimes;)
  {
    readphoto(20);
    if (hosei >= 0) {
      right(speed);
      left(speed + hosei);
    }

    if (hosei < 0) {
      right(speed - hosei);
      left(speed);
    }

  }
  Serial.println("LINETRACE IS NOW BROKEN");
}




int bgoal(int goalspeed) {
  int gotimes = 0;
  crossnumber = 0;
  //gotimesは、自分がどれだけ前進したか（HIGHLOWの往復を何回行ったか）を記録する//

  for (;;) {
    digitalWrite(lwheel, HIGH);
    digitalWrite(rwheel, HIGH);
    delay(goalspeed);
    digitalWrite(lwheel, LOW);
    digitalWrite(rwheel, LOW);
    delay(goalspeed);
    gotimes++;

    readcross(20, 1000);
    if (crossnumber == 1) {
      break;
    }
  }

  Serial.println("BROKEN");

  //車輪を逆回転の状態にセットする//

  digitalWrite(fanpin, LOW);
  //ファンを停止し、ボールを落下させる//
  delay(500);

  #if 0
  back(10, 100);
  for(;;)
  {
    digitalWrite(lwheel, HIGH);
    digitalWrite(rwheel, HIGH);
    delay(goalspeed);
    digitalWrite(lwheel, LOW);
    digitalWrite(rwheel, LOW);
    delay(goalspeed);
    readphoto(goalspeed);

    if(rdata_o < 50)
    {
      break;
    }
  }
  straight(10, 30);
  #endif

  #if 1
  int i;
  digitalWrite(rwheel_a, HIGH);
  digitalWrite(lwheel_a, LOW);

  for (i = 0; i < gotimes - 78; i++) {
    digitalWrite(lwheel, HIGH);
    digitalWrite(rwheel, HIGH);
    delay(goalspeed);
    digitalWrite(lwheel, LOW);
    digitalWrite(rwheel, LOW);
    delay(goalspeed);
  }
  //来た時と同じ回数だけ、車輪を逆回転させる//
  digitalWrite(rwheel_a, LOW);
  digitalWrite(lwheel_a, HIGH);
  //車輪の状態をもとに戻す//
  #endif

  rcheckcrossb(6);
  //180°回転し、元の場所へ//
}
//blue goalの略。手前から見て2番目の十字（青いゴールから最寄りの十字）を超えたタイミングで使うことを想定しているプログラム//
//前進し、ゴールの前でラインを検知してボールを手放す。その後、進んだのと同じ分だけ後退し、十字に沿うように180°方向転換する//


int bgoal2(int goalspeed) {
  int gotimes = 0;
  crossnumber = 0;
  //gotimesは、自分がどれだけ前進したか（HIGHLOWの往復を何回行ったか）を記録する//

  for (;;) {
    digitalWrite(lwheel, HIGH);
    digitalWrite(rwheel, HIGH);
    delay(goalspeed);
    digitalWrite(lwheel, LOW);
    digitalWrite(rwheel, LOW);
    delay(goalspeed);
    gotimes++;

    crossnumber = 0;
    readcross(18, 1000);

    if (crossnumber == 1) {
      break;
    }
  }

  Serial.println("BROKEN");

  //車輪を逆回転の状態にセットする//

  digitalWrite(fanpin, LOW);
  //ファンを停止し、ボールを落下させる//
  delay(600);

  int i;
  digitalWrite(rwheel_a, HIGH);
  digitalWrite(lwheel_a, LOW);


  for (i = 0; i < gotimes - 150; i++) {
    digitalWrite(lwheel, HIGH);
    digitalWrite(rwheel, HIGH);
    delay(goalspeed);
    digitalWrite(lwheel, LOW);
    digitalWrite(rwheel, LOW);
    delay(goalspeed);
  }
  //来た時と同じ回数だけ、車輪を逆回転させる//
  digitalWrite(rwheel_a, LOW);
  digitalWrite(lwheel_a, HIGH);

   lturn(10, 150);
    for(;;)
    {
      lturn(10, 15);
      readphoto(10);
      if((ldata_i) < 50)
      {
        break;
      }
    }
  ball++;

}



int ygoal(int goalspeed) {
  rcheckcrossy(10);
  crossnumber = 0;
  //gotimesは、自分がどれだけ前進したか（HIGHLOWの往復を何回行ったか）を記録する//

  for (;;) {
    digitalWrite(lwheel, HIGH);
    digitalWrite(rwheel, HIGH);
    delay(goalspeed);
    digitalWrite(lwheel, LOW);
    digitalWrite(rwheel, LOW);
    delay(goalspeed);
    ywheeltimes++;

    readcross(20, 1000);
    if (crossnumber == 1) {
      break;
    }
  }

  Serial.println("BROKEN");

  //車輪を逆回転の状態にセットする//

  digitalWrite(fanpin, LOW);
  //ファンを停止し、ボールを落下させる//
  delay(500);


  int i;
  digitalWrite(rwheel_a, HIGH);
  digitalWrite(lwheel_a, LOW);


  for (i = 0; i < ywheeltimes + 24; i++) {
    digitalWrite(lwheel, HIGH);
    digitalWrite(rwheel, HIGH);
    delay(goalspeed);
    digitalWrite(lwheel, LOW);
    digitalWrite(rwheel, LOW);
    delay(goalspeed);
  }
  //来た時と同じ回数だけ、車輪を逆回転させる//
  digitalWrite(rwheel_a, LOW);
  digitalWrite(lwheel_a, HIGH);
  //車輪の状態をもとに戻す//

  rcheckcrossy(10);
  //十字に合わせる//

  #if 0
  rturn(10, 60);
  rcheckcrossb(10);
  //180°回転し、元の場所へ//
  #endif
}
//yellow goalの略。手前から見て1番目の十字（黄ゴールから最寄りの十字）を超えたタイミングで使うことを想定//
//十字に沿って方向転換し、ゴールの手前でラインを検知してボールを手放す。その後、進んだのと同じ分だけ後退し、十字に沿うように方向転換する//



int ygoal2(int goalspeed){
  straight(5, 20);
  lturn(10, 70);
    for(;;)
    {
      lturn(10, 15);
      readphoto(10);
      if((ldata_i) < 50)
      {
        break;
      }
    }
  ywheeltimes = 0;
  linetrace2(10,160);

  for (;;) {
    digitalWrite(lwheel, HIGH);
    digitalWrite(rwheel, HIGH);
    delay(goalspeed);
    digitalWrite(lwheel, LOW);
    digitalWrite(rwheel, LOW);
    delay(goalspeed);
    ywheeltimes++;

    crossnumber = 0;
    readcross(12, 50);
    if (crossnumber == 1) {
      break;
    }
  }

  Serial.print(ywheeltimes);
  Serial.println("BROKEN");

  //車輪を逆回転の状態にセットする//

  digitalWrite(fanpin, LOW);
  //ファンを停止し、ボールを落下させる//
  delay(600);


  int i;
  digitalWrite(rwheel_a, HIGH);
  digitalWrite(lwheel_a, LOW);


  for (i = 0; i < ywheeltimes; i++) {
    digitalWrite(lwheel, HIGH);
    digitalWrite(rwheel, HIGH);
    delay(goalspeed);
    digitalWrite(lwheel, LOW);
    digitalWrite(rwheel, LOW);
    delay(goalspeed);
  }
  //来た時と同じ回数だけ、車輪を逆回転させる//
  digitalWrite(rwheel_a, LOW);
  digitalWrite(lwheel_a, HIGH);

  lturn(10, 95);
    for(;;)
    {
      lturn(10, 15);
      readphoto(10);
      if((ldata_i) < 50)
      {
        break;
      }
    }
  ball++;
}
//動作確認はしていません。checkcross系統の関数の動作があまりにも怪しかったので導入//
int rgoal(int goalspeed) {
  rcheckcrossr(10);
  crossnumber = 0;
  //gotimesは、自分がどれだけ前進したか（HIGHLOWの往復を何回行ったか）を記録する//

  for (;;) {
    digitalWrite(lwheel, HIGH);
    digitalWrite(rwheel, HIGH);
    delay(goalspeed);
    digitalWrite(lwheel, LOW);
    digitalWrite(rwheel, LOW);
    delay(goalspeed);
    rwheeltimes++;

    readcross(20, 1000);
    if (crossnumber == 1) {
      break;
    }
  }

  Serial.println("BROKEN");

  //車輪を逆回転の状態にセットする//

  digitalWrite(fanpin, LOW);
  //ファンを停止し、ボールを落下させる//
  delay(500);


  int i;
  digitalWrite(rwheel_a, HIGH);
  digitalWrite(lwheel_a, LOW);


  #if 1
  for (i = 0; i < rwheeltimes - 50; i++) {
    digitalWrite(lwheel, HIGH);
    digitalWrite(rwheel, HIGH);
    delay(goalspeed);
    digitalWrite(lwheel, LOW);
    digitalWrite(rwheel, LOW);
    delay(goalspeed);
  }
  rturn(10, 190);
  //rwheeltimesに記録されていたのと同じ回数だけ逆回転し、元の場所まで戻る。-50は補正//
  #endif 


  #if 0
  for (i = 0; i < rwheeltimes + 24; i++) {
    digitalWrite(lwheel, HIGH);
    digitalWrite(rwheel, HIGH);
    delay(goalspeed);
    digitalWrite(lwheel, LOW);
    digitalWrite(rwheel, LOW);
    delay(goalspeed);
  }
  //来た時と同じ回数だけ、車輪を逆回転させる//
  digitalWrite(rwheel_a, LOW);
  digitalWrite(lwheel_a, HIGH);
  //車輪の状態をもとに戻す//

  rcheckcrossr(10);
  //十字に合わせる//
  #endif

}
int rcheckcrossb(int checkspeed) {
  rturn(4, 20);

  for (;;) {
    rturn(checkspeed, 10);
    readphoto(10);
    if (ldata_o < 60 && rdata_o < 60 && (ldata_i < 60 || rdata_i < 60)) {
      break;
    }
  }
}
//right check cross blueの略。十字の手前まで後退した状態で使えば、十字に沿うように180°方向転換する//
//「十字の手前」っていうのがすごく微妙……//
int rcheckcrossy(int checkspeed) {
  straight(7, 60);
  lturn(7, 110);
  back(7, 33);
  int i;
  ywheeltimes = 0;

  for (;;) {

    lturn(checkspeed, 8);
    readphoto(8);
    if (ldata_o < 38 && rdata_o < 38 && (ldata_i < 38 || rdata_i < 38)) {
      Serial.println("Start linetrace.");
      for (i = 0; i <= 14000; i++) {
        readphoto(20);
        if (hosei >= 0) {
          right(4);
          left(4 + hosei);
        }

        if (hosei < 0) {
          right(4 - hosei);
          left(4);
        }
      }
      Serial.println(ywheeltimes);
      break;
    }
  }
}
//十字を検出したタイミングで使用すると、十字に沿うように90°方向転換する//
//これも精度が怪しい……//
int rcheckcrossr(int checkspeed) {
  straight(7, 60);
  rturn(7, 110);
  back(7, 33);
  int i;
  rwheeltimes = 0;

  for (;;) {

    rturn(checkspeed, 8);
    readphoto(8);
    if (rdata_o < 38 && (ldata_i < 38 || rdata_i < 38)) {
      Serial.println("Start linetrace.");
      for (i = 0; i <= 14000; i++) {
        readphoto(20);
        if (hosei >= 0) {
          right(4);
          left(4 + hosei);
        }

        if (hosei < 0) {
          right(4 - hosei);
          left(4);
        }
      }
      Serial.println(ywheeltimes);
      break;
    }
  }
}
void readcolor(void) {
  unsigned short h, l, r, g, b;
  float red, green, blue, average;
  float red2, green2, blue2;
  Wire.beginTransmission(S11059_ADDR);
  Wire.write(0x03);
  Wire.endTransmission();
  Wire.requestFrom(S11059_ADDR, 6);
  //センサーが読み取ったデータを保存するレジスタを指定しています//

  if (Wire.available() > 0) {
    h = Wire.read();
    l = Wire.read();
    r = h << 8 | l;

    h = Wire.read();
    l = Wire.read();
    g = h << 8 | l;

    h = Wire.read();
    l = Wire.read();
    b = h << 8 | l;

    red = r;
    green = g * 0.93;
    blue = b * 1.7;
    //それぞれの値に補正をかけて、平均値がおおよそ同じくらいになるようにしています。補正値は適当です。根拠はないです。//

    average = (red + green + blue) / 3;
    red2 = red / average;
    green2 = green / average;
    blue2 = blue / average;
    //平均で割り、比の値として出力します。1より大きければ、その色が強いということです//

    Serial.print("r:");
    Serial.print(red);
    Serial.print(" g:");
    Serial.print(green);
    Serial.print(" b:");
    Serial.println(blue);

    Serial.print("r':");
    Serial.print(red2);
    Serial.print(" g':");
    Serial.print(green2);
    Serial.print(" b':");
    Serial.println(blue2);

  #if 1
    if (1.48 < blue2) {
      Serial.println("Blue");
      color = 'B';
      Serial.println(color);
    }

    else if (1.33 < red2) {
      Serial.println("Red");
      color = 'R';
      Serial.println(color);

    }

    else if (1.03 < red2 && blue2 < 0.87) {
      Serial.println("Yellow");
      color = 'Y';
      Serial.println(color);

    }

    else {
      Serial.println("Nothing");
      color = 'N';
    }
  #endif
  }
}
//カラーセンサーを読み取る。色に応じて、colorに各色の頭文字を格納する//
int readlight(void) {
  distance_1 = Vcc * analogRead(lightpin) / 1023;  //(5.0V*センサ数値/1023)1023は5V入力時の値
  distance_2 = 26.549 * pow(distance_1, -1.2091);  //距離換算
  if (distance_2 > 80) {
    distance_2 = 80;
  }

  Serial.println(distance_2);
}
//ライトセンサーを読み取り、読み取った結果をdistance_2に格納//
int readphoto(int photospan) {
  if (millis() - last_phototime > photospan) {
    ldata_o = analogRead(lsensor_o);
    ldata_i = analogRead(lsensor_i);
    rdata_i = analogRead(rsensor_i);
    rdata_o = analogRead(rsensor_o);
    //それぞれのデータが同程度になるように、補正値をかけています。が、場所によって全然変わってしまうので、うーん……。//
    //補正値かけても、別の場所に行くとすぐ値が変動してしまう……。//

  #if 0
    Serial.print(ldata_o);
    Serial.print(":");
    Serial.print(ldata_i);
    Serial.print(":");
    Serial.print(rdata_i);
    Serial.print(":");
    Serial.println(rdata_o);
  #endif

    last_phototime = millis();
    pid(&photospan);
  #if 1
    Serial.print("補正値は");
    Serial.println(hosei);
  #endif
  }
}
//photospan周期でフォトリフレクターの値を読み取り、dataに保存する//
//ついでにpidの補正値を、photospan周期で求める//
int readcross(int crossspan1, int crossspan2) {
  if (millis() - last_crosstime1 > crossspan1) {
    ldata_o = analogRead(lsensor_o);
    rdata_o = analogRead(rsensor_o);
    #if 0
    Serial.print(ldata_o);
    Serial.print(":");
    Serial.println(rdata_o);
    #endif
    last_crosstime1 = millis();

    if (ldata_o < 42 && rdata_o < 42 && millis() - last_crosstime2 > crossspan2) {
      crossnumber++;
      last_crosstime2 = millis();
      Serial.print("crossnumber =");
      Serial.println(crossnumber);
    }
  }
}
//十字の数を読み取り、cross_numberに保存していく//
//corssspan1の間隔で、センサーを確認する。crossspan2は、一度十字を読み取ってからのクールタイム。これがないと同じ十字を複数回読み取り続けるので……//
int pid(int *p_photospan) {
  diff = (float)(rdata_i - ldata_i);
  sum_diff += diff * (*p_photospan) * 0.001;                 //積分の式。photospan(= *p_photospan)が微小時間dtに該当。マイクロ秒から秒に戻し、それをdiffにかけて足し合わせることで積分する//
  derivative = (diff - last_diff) / (*p_photospan * 0.001);  //微分の式。photospanが微小時間dtに該当。マイクロ秒から秒に戻す//
  last_diff = diff;

  hosei = (int)(float(diff) * 0.38 + sum_diff * 0.008 + derivative * 0.0001);

  #if 0
  hosei = last_hosei * 0.97 + hosei * 0.03;
  last_hosei = hosei;
  #endif

  if (hosei <= 10 && hosei >= -10) {
    hosei = 0;
  }

  #if 0
  Serial.println(hosei);
  #endif

  return hosei;
}
//pid制御における、モーターの回転速度の補正値を求める//

