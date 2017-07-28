#include <ESP8266WiFi.h>
#include <ArduinoJson.h>
#include <math.h>
//This is the code for the working of the ESP8266 board.
const char* ssid     = "MaRS_2";
const char* password = "swarmiitr";

const char* host = "192.168.0.124";
const int httpPort = 8080;

#define PI 3.1416
#define DEG 180

//#define DEBUG

#define NUMBER_OF_BOTS 4
//  Red , Green , Blue , White
#define CUR_BOT 3


String  colors [] = { "Red", "Green", "Blue", "White"};
//when upload for red do -45 in DEF_R_PWM

#define DEF_L_PWM  350
#define DEF_R_PWM  350

//motor1
#define RIGHTPWM D6
#define RIGHTDIR1 D4
#define RIGHTDIR2 D5

//motor2
#define LEFTPWM D7
#define LEFTDIR1 D1
#define LEFTDIR2 D3

double x[NUMBER_OF_BOTS], y[NUMBER_OF_BOTS], angle[NUMBER_OF_BOTS], dist[NUMBER_OF_BOTS];
double left_pwm, right_pwm;

// array storing dist of cur bot from other bots
double bot_dist[NUMBER_OF_BOTS], init_bot_dist[NUMBER_OF_BOTS];
// pid angle
double Kp_angle = 1.1, Kd_angle = 0.3, Ki_angle = 0.0;
double prev_angle = 0, error_angle = 0, derror_angle = 0, sumerror_angle = 0;
double correction_angle = 0;


//double xf[], yf[], af[] ;
double xf [] = { 100.0, 100.0, 500, 500 } ;
double yf [] = {100, 400, 100, 400 };
bool is_assigned[NUMBER_OF_BOTS] = { false };
int pos_to_go [NUMBER_OF_BOTS] ;

int leader = 30 ;

// flag variables
int flag = 0;

int req = 0 ;
int prev_req = 0 ;


void setup() {
  Serial.begin(38400);

  pinMode(RIGHTPWM, OUTPUT);
  pinMode(RIGHTDIR1, OUTPUT);
  pinMode(RIGHTDIR2, OUTPUT);
  pinMode(LEFTPWM, OUTPUT);
  pinMode(LEFTDIR1, OUTPUT);
  pinMode(LEFTDIR2, OUTPUT);

  // We start by connecting to a WiFi network
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  // set the initial values of pwm
  set_pwm();
  // the bot has to be stationery before processing any code
  stop1();

  // if wifi is not connected keep trying
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi connected");
  Serial.print("IP address:   ");
  Serial.println(WiFi.localIP());


  updateReadings();
  while (req == 0 ) {
    updateReadings();
    Serial.println("command not received");
  }
  prev_req = req  ;

}

/*   meaning of return values for updateReadings()
    -1 : client not connected
    -2 : client timeout
    -3 : json parse failed
    0  : all readings are available
    1  : atleast one value has -1
    2  : the readings of current bot are -1
*/
int  updateReadings() {

  int retvalue = 0;

  Serial.print("connecting to ");
  Serial.println(host);

  // Use WiFiClient class to create TCP connections
  WiFiClient client;
  if (!client.connect(host, httpPort)) {
    Serial.println("connection failed");
    Serial.println();
    stop1();
    return -1 ;
  }

  // This will send the request to the server and then expect the data in the reply
  client.println("GET /getdata HTTP/1.1\r\n");

  unsigned long timeout = millis();
  while (client.available() == 0) {
    if (millis() - timeout > 2000 ) {
      Serial.println(">>> Client Timeout !");
      client.stop();
      return -2 ;
    }
  }

  String line = "";
  // Read all the lines of the reply from server and print them to Serial
  while (client.available()) {
    line = client.readStringUntil('\r');
    //      Serial.println(line);

  }


  Serial.println("closing connection");
  client.stop(); // stop the client since you have got your single line


  DynamicJsonBuffer  jsonBuffer;
  JsonObject& root = jsonBuffer.parseObject(line);
  // Test if parsing succeeds.
  if (!root.success()) {
    Serial.println("parseObject failed");
    return -3 ;
  }
  Serial.println("parsing success");
  for (int i = 0; i < NUMBER_OF_BOTS ; i++) {
    x[i] = root["xdata"][i] ;
    y[i] = root["ydata"][i];
    angle[i] =  root["angle"][i];
    req = root["command"];
    // due to some random things between server and client this value has to get a negative sign
    angle[i] =  - ( angle[i] ) ;
    if (( x[i] == -1) || ( y[i] == -1) ) {
      retvalue = 1 ;
    }

#ifdef DEBUG
    Serial.print(colors[i]);
    Serial.print(" ");
    Serial.print(x[i]);
    Serial.print(" ");
    Serial.print(y[i]);
    Serial.print(" ");
    Serial.println(angle[i]);
    Serial.print("color of leader is ");
    Serial.println(colors[leader]);
#endif
  }
  if (( x[CUR_BOT] == -1) || ( y[CUR_BOT] == -1) ) {
    retvalue = 2 ;
  }
  return retvalue  ;
}

void set_pwm() {
  left_pwm = DEF_L_PWM ;
  right_pwm = DEF_R_PWM ;
}

void stop1 () {
  digitalWrite(RIGHTDIR1, LOW);
  digitalWrite(RIGHTDIR2, LOW);
  digitalWrite(LEFTDIR1, LOW);
  digitalWrite(LEFTDIR2, LOW);

  analogWrite(RIGHTPWM, 0);
  analogWrite(LEFTPWM, 0);
}

#define ROTATE_PWM 320

void run_forward() {
  digitalWrite(RIGHTDIR1, HIGH);
  digitalWrite(RIGHTDIR2, LOW);
  digitalWrite(LEFTDIR1, HIGH);
  digitalWrite(LEFTDIR2, LOW);

  analogWrite(RIGHTPWM, right_pwm  );
  analogWrite(LEFTPWM, left_pwm );
}

void rotate_anticlockwise() {
  Serial.println("inside rotate_ANTIclockwise");
  digitalWrite(RIGHTDIR1, HIGH);
  digitalWrite(RIGHTDIR2, LOW);
  digitalWrite(LEFTDIR1, LOW);
  digitalWrite(LEFTDIR2, HIGH);

  analogWrite(RIGHTPWM, ROTATE_PWM  );
  analogWrite(LEFTPWM,  ROTATE_PWM  );

}

void rotate_clockwise() {
  Serial.println("inside rotate_clockwise");
  digitalWrite(RIGHTDIR1, LOW);
  digitalWrite(RIGHTDIR2, HIGH);
  digitalWrite(LEFTDIR1, HIGH);
  digitalWrite(LEFTDIR2, LOW);

  analogWrite(RIGHTPWM, ROTATE_PWM  );
  analogWrite(LEFTPWM, ROTATE_PWM );
}


void assignPoints() {
  double distance  = 0 ;
  Serial.println("inside assign pionts function");

  for (int i = 0 ; i < NUMBER_OF_BOTS ; i++ ) {
    double min_dist = 1e32 ;
    int min_bot = 1e32 ;
    for (int j = 0 ; ( j < NUMBER_OF_BOTS ); j++ ) {
      distance   = ((xf[i] - x[j]) * (xf[i] - x[j])) + ( (yf[i] - y[j]) * (yf[i] - y[j])) ;
      if (distance < min_dist  && ( !( is_assigned[j] ) )  ) {
        min_dist = distance ;
        min_bot  = j ;
      }
    }
    is_assigned[min_bot] = true ;
    pos_to_go [min_bot] = i ;
    Serial.println(min_bot);
  }

  for (int i = 0 ; i < NUMBER_OF_BOTS ; i++ ) {
    Serial.print(colors[i]);
    Serial.print("  ") ;
    Serial.println(pos_to_go[i]);
  }

}

void loop () {

  int optim = 0;
  int flag = 0 ;
  updateReadings();
  while (req == prev_req  ) {

    updateReadings();
    Serial.println("command not updated");
  }
  prev_req = req  ;
  switch (req) {

    case 1://square

      xfinit(150, 150, 400, 400 );
      yfinit(150, 400, 150, 400 );
      break ;
    case 2://rectangle
      xfinit (150, 150, 550, 550 ) ;
      yfinit ( 150, 400, 150, 400 );
      break ;
    case 3://line
      xfinit (140, 280, 420, 560) ;
      yfinit (100, 200,  300, 400);
      break ;
    //    case 4: { //switch position, call after making line or square
    //        int temp = 0 ;
    //
    //        temp =  xf[0];
    //        xf[0] = xf[1];
    //        xf[1] = xf[2];
    //        xf[2] = xf[3];
    //        xf[3] = temp;
    //
    //        temp =  yf[0];
    //        yf[0] = yf[1];
    //        yf[1] = yf[2];
    //        yf[2] = yf[3];
    //        yf[3] = temp;
    //        optim = 1;
    //        break ;
    //      }
    case 5: //rotate at its own position
      rotate_clockwise();
      delay(5000);
      break ;
    case 0 :
      stop1() ;
      flag = 1 ;
      break ;
    default :
      stop1 () ;

  }


  //  if (optim == 0) {

  int flag1 = 0 ;

  while ( flag1 == 0 ) {
    if ( updateReadings() == 0 ) {
      Serial.println("life is fucked ");
      assignPoints();
      flag1 = 1 ;
    }
  }
  xf[CUR_BOT] = xf[pos_to_go [CUR_BOT] ];
  yf[CUR_BOT] = yf[pos_to_go [CUR_BOT] ];
  //  }
  Serial.println("before life geets fucked ");
  Serial.print (xf[CUR_BOT]) ;
  Serial.print("  ");
  Serial.println(yf[CUR_BOT]);


  if (flag != 1 )
    run_pid ();
}

void run_pid () {

  //#ifdef DEBUG
  Serial.println("inside run_pid function ");
  //  Serial.print("X final   X current ");
  //  Serial.print(xf[CUR_BOT]);
  //  Serial.print("  ");
  //  Serial.println(x[CUR_BOT]);
  //
  //  Serial.print("Y final   Y current ");
  //  Serial.print(yf[CUR_BOT]);
  //  Serial.print("  ");
  //  Serial.println(y[CUR_BOT]);
  //#endif

  set_orientation () ;

  while ( 1 ) {

    updateReadings ();

    // if the request changes before the current process is complete stop from the current process
    if (req != prev_req )
      break ;

    for (int p = 0 ; p < NUMBER_OF_BOTS ; p++  ) {
      double distance_bots = ( (x[CUR_BOT] - x[p]) * (x[CUR_BOT] - x[p]) + ( y[CUR_BOT] - y[p] ) * (y[CUR_BOT] - y[p]) ) ;
      if ( ( distance_bots < 10000 ) && ( distance_bots != 0 ) && (CUR_BOT > p) ) {
        delay (3000) ;
        break ;
      }
    }

    double theta = atan2 ( (yf[CUR_BOT] - y[CUR_BOT]),  (xf[CUR_BOT] - x[CUR_BOT]) );

    theta = (theta * DEG ) / PI ;

    double dis = (x[CUR_BOT] - xf[CUR_BOT]) * (x[CUR_BOT] - xf[CUR_BOT]) + (y[CUR_BOT] - yf[CUR_BOT]) * (y[CUR_BOT] - yf[CUR_BOT]);

    if (( (angle[CUR_BOT] - theta) < DEG ) && ( (angle[CUR_BOT] - theta) > -DEG ) ) {
      error_angle = angle[CUR_BOT] - theta;
    }
    else if ( (angle[CUR_BOT] - theta ) > DEG ) {
      error_angle = -( (2 * DEG ) - (angle[CUR_BOT] - theta)) ;
    }
    else if ( ( angle[CUR_BOT] - theta ) < -DEG )
    {
      error_angle =  (2 * DEG ) + angle[CUR_BOT] - theta;
    }
    //getting error

#ifdef DEBUG
    Serial.print(" theta angle  ");
    Serial.print(theta);
    Serial.print("  ");
    Serial.println(error_angle);

    Serial.print("Error :  ");
    Serial.print(error_angle);
    Serial.print("  distance :   ") ;
    Serial.println(dis);
#endif DEBUG

    if (dis < 1500 )
    {
      Serial.println("stopping run_pid because dis < 1500 ");
      stop1();
      break ;
    }
    else
    {
      derror_angle = error_angle - prev_angle;
      sumerror_angle += error_angle ;
      double pwm = Kp_angle * error_angle + Kd_angle * derror_angle + Ki_angle * sumerror_angle ;
      left_pwm = DEF_L_PWM +  pwm;
      right_pwm = DEF_R_PWM - pwm ;
      run_forward();
      Serial.print("LeftPwm :  ");
      Serial.print(left_pwm);
      Serial.print(" ,  Rightpwm : " );
      Serial.println(right_pwm);
    }
    prev_angle = error_angle;

  }

}

void set_orientation() {

  double req_angle =  atan2 ( (yf[CUR_BOT] - y[CUR_BOT]),  (xf[CUR_BOT] - x[CUR_BOT]) );

  req_angle = (req_angle * DEG ) / PI ;

  while ( error_angle > 30 || error_angle < -30 ) {

    // stop the bot if the readings of the current bot are -1
    while (updateReadings() == 2 )
      stop1() ;


    error_angle = angle[CUR_BOT] - req_angle;

    // logic to determine the correct angle
    if (( (error_angle) < DEG ) && ( (error_angle) > -DEG ) ) {
      error_angle = error_angle;
    }
    else if ( (error_angle ) > DEG) {
      error_angle = -( (2 * DEG ) - (error_angle)) ;
    }
    else if ( ( error_angle ) < -DEG)
    {
      error_angle =  (2 * DEG  ) + error_angle;
    }

    Serial.print ("error   ");
    Serial.print (error_angle);

    if (error_angle < 0 )
      rotate_anticlockwise();
    else
      rotate_clockwise();

  }
  set_pwm();
  stop1();
}
void xfinit(int i, int j, int k, int l)
{
  xf[0] = i;
  xf[1] = j;
  xf[2] = k;
  xf[3] = l;
}
void yfinit(int i, int j, int k, int l)
{
  yf[0] = i;
  yf[1] = j;
  yf[2] = k;
  yf[3] = l;
}

