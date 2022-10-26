#include <PS4Controller.h>
​
void setup()
{
    Serial.begin(115200);
    PS4.begin(38:00:25:2d:34:d4);
    Serial.println(Ready.);
    Serial.write(' ');
}
​
int8_t ilx,ily;
uint8_t lx,ly;
int moving=0;
​
char r1_or_l1=0;
char pre_r1_or_l1=0;
​
char hold=0;
​
char push_l2_r2=0;
char pre_push_l2_r2=0;
​
char pre_lr=0;
char pre_ud=0;
​
char pre_tc=0;
char pre_sc=0;
​
void loop()
{
    // Below has all accessible outputs from the controller
    if(PS4.isConnected()) {
     
      if ( PS4.data.button.up ){
          //Serial.println(Up Button);
          Serial.write('e');
          pre_ud=1;
      }
      else if ( PS4.data.button.down ){
          //Serial.println(Down Button);
          Serial.write('g');
          pre_ud=1;
      }
      else if(pre_ud==1){
          Serial.write('F');
          pre_ud=0;
      }
​
      if ( PS4.data.button.options ){
          //Serial.println(Options Button);
          Serial.write('N');
          hold=1;
      }
      
      if ( PS4.data.button.left ){
          //Serial.println(Left Button);
          Serial.write('k');
          pre_lr=1;
          //hold=1;
      }
      else if ( PS4.data.button.right ){
          //Serial.println(Right Button);
          Serial.write('m');
          pre_lr=1;
          //hold=0;
      }
      else if(pre_lr==1){
        //if(hold==0){
          Serial.write('L');
          pre_lr=0;
        //}
      }
   /*     
      if ( PS4.data.button.upright ){
          //Serial.println(Up Right);
      }
          
      if ( PS4.data.button.upleft ){
          //Serial.println(Up Left);
      }
      
      if ( PS4.data.button.downleft ){
          //Serial.println(Down Left);
      }
      
      if ( PS4.data.button.downright ){
          //Serial.println(Down Right);
      }
      */
        
      if ( PS4.data.button.triangle )
      {
          Serial.write('o');
          pre_tc=1;
          //Serial.println(Triangle Button);
          //delay(10);
      }
      else if ( PS4.data.button.cross ){
          Serial.write('q');
          pre_tc=1;
          //Serial.println(Cross Button);
      }
      else if(pre_tc==1){
          Serial.write('P');
          pre_tc=0;
      }
          
      if ( PS4.data.button.circle ){
          Serial.write('u');
          pre_sc=1;
      }
      else if ( PS4.data.button.square ){
          Serial.write('s');
          pre_sc=1;
      }
      else if(pre_sc==1){
          Serial.write('T');
          pre_sc=0;
      }
        
      
      if ( PS4.data.button.l3 ){
          Serial.write('y');
      }
      
      if ( PS4.data.button.r3 ){
          Serial.write('w');
      }
      else{
        //Serial.write('y');
      }
      /*
      if ( PS4.data.button.share ){
      }
      
      
        
      if ( PS4.data.button.ps ){
      }
​
      
      if ( PS4.data.button.touchpad ){
      }
      */
​
      push_l2_r2=0;
      if ( PS4.data.button.l2 ) {
        if(PS4.data.analog.button.l2>15){
          Serial.write('A');
          push_l2_r2=1;
          pre_push_l2_r2=1;
        }
          //Serial.print(l2 button at );
          //Serial.println(PS4.data.analog.button.l2, DEC);
      }
      if ( PS4.data.button.r2 ) {
        if(PS4.data.analog.button.r2>15){
          Serial.write('C');
          push_l2_r2=1;
          pre_push_l2_r2=1;
        }
          //Serial.print(r2 button at );
          //Serial.println(PS4.data.analog.button.r2, DEC);
      }
​
      if(push_l2_r2==0 && pre_push_l2_r2==1){
         Serial.write('V');
         pre_push_l2_r2=0;
      }
/*
      if ( PS4.event.analog_move.stick.lx ) {
        
          //Serial.print(Left Stick x at );
          //Serial.println(PS4.data.analog.stick.lx, DEC);
          
      }
      if ( PS4.event.analog_move.stick.ly ) {
          //Serial.print(Left Stick y at );
          //Serial.println(PS4.data.analog.stick.ly, DEC);
      }
      if ( PS4.event.analog_move.stick.rx ) {
          //Serial.print(Right Stick x at );
          //Serial.println(PS4.data.analog.stick.rx, DEC);
      }
      if ( PS4.event.analog_move.stick.ry ) {
          //Serial.print(Right Stick y at );
          //Serial.println(PS4.data.analog.stick.ry, DEC);
      }
*/
      r1_or_l1=0;
​
      if ( PS4.data.button.l1 ){
          //Serial.println(l1 Button);
          Serial.write('a');
          r1_or_l1=1;
          pre_r1_or_l1=1;
      }
      else if ( PS4.data.button.r1 ){
          //Serial.println(r1 Button);
          Serial.write('c');
          r1_or_l1=1;
          pre_r1_or_l1=1;
      }
      if(r1_or_l1==0 && pre_r1_or_l1 == 1){
          Serial.write('X');
          pre_r1_or_l1=0;
      }
​
     /*
      ilx=PS4.data.analog.stick.lx;
      ily=PS4.data.analog.stick.ly;
      
      if(ilx*ilx+ily*ily>49){
        Serial.write('i');
        if(ilx=='a' || ilx=='c' || ilx=='e' || ilx=='F' || ilx=='g' || ilx=='i' || ilx=='k' || ilx=='L' || ilx=='m' || ilx=='o' || ilx=='P' || ilx=='q' || ilx=='s' || ilx=='T' || ilx=='u' || ilx=='w' || ilx=='y' || ilx=='A' || ilx=='N' || ilx=='C' || ilx=='V' || ilx=='X')ilx++;
        Serial.write(ilx);
        if(ily=='a' || ily=='c' || ily=='e' || ily=='F' || ily=='g' || ily=='i' || ily=='k' || ily=='L' || ily=='m' || ily=='o' || ily=='P' || ily=='q' || ily=='s' || ily=='T' || ily=='u' || ily=='w' || ily=='y' || ily=='A' || ily=='N' || ily=='C' || ily=='V' || ily=='X')ily++;
        Serial.write(ily);
        //moving=1;
      }
      else if(r1_or_l1==0){
        Serial.write('i');
        Serial.write(0);
        Serial.write(0);
​
        //moving=0;
      }
      */
      
​
      
     if (PS4.data.status.charging){
        //Serial.println(The controller is charging);
     }
        
     if (PS4.data.status.audio){
        //Serial.println(The controller has headphones attached);
     }
     
     if (PS4.data.status.mic){
        //Serial.println(The controller has a mic attached);
     }
     
​
     /*
     Serial.print(Battey = );
     Serial.print(PS4.data.status.battery, DEC);
     Serial.println( / 16);
​
     Serial.println();
     */
     
     // This delay is to make the Serial Print more human readable
     // Remove it when you're not trying to see the output
     //delay(1000);
     
​
    }
    else{
      //Serial.write('A');
      Serial.write('F');
      Serial.write('P');
      Serial.write('L');
      Serial.write('T');
      Serial.write('V');
      Serial.write('X');
      Serial.write('y');
      //Serial.write('i');
      //Serial.write(0);
    }
    
}
