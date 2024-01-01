  #include <PID_v1.h>
  #include <Encoder.h>
  #include <ros.h>
  #include <geometry msgs/Twist.h>

  double kp1 = 0.4;
  double ki1 = 0;
  double kd1 = 0;
  double starttime =0;

  float x;
  float z;
  ros::NodeHandle nh;

  float encoder0Diff;
  float encoder1Diff;

  float encoder0Error;
  float encoder1Error;

  float encoder0Prev;
  float encoder1Prev;
  

  double Setpoint1 = 0, Input1 = 0, Output1 = 0, Output1a; // PID variables
  PID my_PID1(&Input1, &Output1, &Setpoint1, kp1, ki1, kd1, DIRECT); // PID Setup

  double kp2 = 0.3;
  double ki2 = 0;
  double kd2 = 0;

  double Setpoint2 = 0, Input2 = 0, Output2 = 0, Output2a; // PID variables
  PID my_PID2(&Input2,&Output2, &Setpoint2, kp2, ki2, kd2, DIRECT); // PID Setup

  double r=0.05; // bán kính bánh xe 
  double encoder_res = 2400; // encoder resolution

  #define ENA 10 // PWM outputs to L298N H-Bridge motor driver module
  const int IN1=11;
  const int IN2=12;

  #define ENB 7 // PWM outputs to L298N H-Bridge motor driver module
  const int IN3=8;
  const int IN4=9;
   
  Encoder myEnc1(2, 4);
  Encoder myEnc2(3, 5);

  // Wheel encoder interrupts
  #define encoder0PinA 2    // encoder 1
  #define encoder0PinB 3

  #define encoder1PinA 18   // encoder 2 
  #define encoder1PinA 19

  volatile long encoder0Pos = 0; // encoder 1
  volatile long encoder1Pos = 0; // encoder 2
  
  
  void velCallback(const geometry_msgs::Twist& vel){
    x = vel.linear.x;
    z = vel.angular.z;}
  ros::Subscriber<geometry_msgs::Twist> sub ("cmd_vel" , velCallback);
  geometry_msgs::Vector3Stamped speed_msg;                                //create a "speed_msg" ROS message
  ros::Publisher speed_pub("speed", &speed_msg);
  

  void Drive_MotorA(int out) {
      //Serial.println(out);
      if (out > 0){
        digitalWrite(IN1,LOW);
        digitalWrite(IN2,HIGH);
        analogWrite(ENA, out);
      }
      else {
         digitalWrite(IN1,HIGH);
         digitalWrite(IN2,LOW);
         analogWrite(ENA, abs(out));
      }
   }     

   void Drive_MotorB(int out) {
      //Serial.println(out);
      if (out > 0){
         digitalWrite(IN3,HIGH);
         digitalWrite(IN4,LOW);
         analogWrite(ENB, out);
      }
      else {
         digitalWrite(IN3,LOW);
         digitalWrite(IN4,HIGH);
         analogWrite(ENB, abs(out));
      }
   }

   double convert(double tocdo) {
      double y = ((60*tocdo))/((2*3.14*r)); // vòng/phút
      //Serial.println(y);
      double pulse = (y*encoder_res)/(60*100); // convert to pulse/10ms
      //Serial.println(pulse);
      return pulse;
   }

   void setup() {

      pinMode(IN1, OUTPUT);  // motor PWM pins
      pinMode(IN2, OUTPUT);
      pinMode(IN3, OUTPUT);
      pinMode(IN4, OUTPUT);

      digitalWrite(ENA, HIGH);
      digitalWrite(ENB, HIGH);

//      moi add
      //Serial1. begin (115200) ;
      nh.initNode ();
      nh.subscribe (sub); 
      nh.advertise(speed_pub);   

      my_PID1.SetMode(AUTOMATIC);
      my_PID1.SetSampleTime(1);
      my_PID1.SetOutputLimits(-255, 250);
      

      my_PID2.SetMode(AUTOMATIC);
      my_PID2.SetSampleTime(1);
      my_PID2.SetOutputLimits(-252, 255);
      
      starttime = 0;
      Input1 = myEnc1.read(); // read value of original position
      Input2 = myEnc2.read(); // read value of original position
      
   }

   void loop() {
      double nowtime = millis();
      if((nowtime - starttime) >= 10) { // start timed loop for everything else
        starttime = nowtime;

        speed_left = x - (z*0.15);      // speed wheel left (0.15 distance among 1/2 two wheel)
        speed_right = x + (z*0.15);

        encoder0Diff = encoder0Pos - encoder0Prev; // Get difference between ticks to compute speed
        encoder1Diff = encoder1Pos - encoder1Prev;
      
        speed_act_left = encoder0Diff/24;                    
        speed_act_right = encoder1Diff/24; 
      
        encoder0Error = (demand_speed_left*24)-encoder0Diff; // 3965 ticks in 1m = 39.65 ticks in 10ms, due to the 10 millis loop
        encoder1Error = (demand_speed_right*24)-encoder1Diff;
      
        encoder0Prev = encoder0Pos; // Saving values
        encoder1Prev = encoder1Pos;

        left_setpoint = demand_speed_left*24;  //Setting required speed as a mul/frac of 1 m/s
        right_setpoint = demand_speed_right*24;
      
        left_input = encoder0Diff;  //Input to PID controller is the current difference
        right_input = encoder1Diff;

        Input1 = myEnc1.read();
        my_PID1.Compute();

        Input2 = myEnc2.read();
        //Serial.println(input2);
        my_PID2.Compute();
        //Serial.println(Output1);
        Drive_MotorA(Output1);
        Drive_MotorB(Output2);

   
      }

      publishSpeed(10);   
      nh.spinOnce () ;

      // Serial1.print (x) ; 
      // Serial1.print (" , ");
      // Serial1.println (z);
   }

  void publishSpeed(double time) {
    speed_msg.header.stamp = nh.now();      //timestamp for odometry data
    speed_msg.vector.x = speed_act_left;    //left wheel speed (in m/s)
    speed_msg.vector.y = speed_act_right;   //right wheel speed (in m/s)
    speed_msg.vector.z = time/1000;         //looptime, should be the same as specified in LOOPTIME (in s)
    speed_pub.publish(&speed_msg);
  //  nh.loginfo("Publishing odometry");
  }

