// import osc libraries
import netP5.*;import oscP5.*;
// import Kinect Libraries (Capital K)
import org.openkinect.freenect.*;import org.openkinect.processing.*;import org.openkinect.tests.*;
// Declare kinect object
Kinect kinect;
// set up default values
int angle = 30;int skip = 10;int max=700;int transport=0;int tremain;int blen=1;;
// set records
int cd;int hy;int ly;int lx;int rx;int icd;
// subsidury coordinates
int cdx;int cdy; int hyx;int hyd;int lyx;int lyd;int lxy;int lxd;int rxy;int rxd;

float tr;

// prepare to convert values
float[] depthLookUp = new float[2048];
// Set up OSC Communication
OscP5 oscP5;NetAddress myRemoteLocation;
// Depth image
PImage depthImg;
void setup()  {size(1550, 1000);  kinect = new Kinect(this);kinect.initDepth();
kinect.activateDevice(1);kinect.enableMirror(true);depthImg = new PImage(kinect.width, kinect.height);
/* start oscP5, listening for incoming messages at port 12000 */
  oscP5 = new OscP5(this,12000); myRemoteLocation = new NetAddress("127.0.0.1",12001);

}
// #############################################################
void draw()  {

  

  
if (transport == 1){background(255,30,30);fill(200);textSize(40);text("Recording",430,100);text("Time Remaining: "+tr+" Seconds",430,400);
 fill(255);line(0.1*640,120,0.1*640,150);line(0.1*640+(blen-tremain)*640*0.9/blen,120,0.1*640+(blen-tremain)*640*0.9/blen,150);    
 }println(blen);
  if (transport ==0){background(255);}
// create an image from depth values and store raw depth values
PImage img = kinect.getDepthImage();int[] rawDepth = kinect.getRawDepth();
cd=2048;hy=0;ly=img.height;lx=img.width;rx=0;
cdx=700;cdy=900;
icd = 2048; // invisible depth record for approach speed estimation.


  // Display info
 
  line(240,0,240,1000);
  textSize(18);
  text("Calibration Shortcuts",20,20);
  line(24,26,198,26);
  textSize(16);
  fill(0,0,200);
  text("Angle Up: Up Arrow",20,50);
  text("Angle Down: Down Arrow",20,70);
  text("Max Depth Up x 10: 0",20,90);
  text("Max Depth Up x 1: 9",20,110);
  text("Max Depth Down x 1: 8",20,130);
    text("Max Depth Down x 10: 7",20,150);

// ----------------------------------------------------------------
// Loop through blocks of pixels for depth values
for (int x = 0; x < img.width; x+=skip) {   
for (int y = 0; y < img.height; y+=skip) {   
// find the index of that pixel
int i = x + y * img.width;
// find depth values
int d = rawDepth[i]; 
if (d < icd){icd = d;}
if (d < max){              // limit depth
if (d<cd){cd=d;cdx=x;cdy=y;}           // set depth record
if (d<cd+40){              // depth range
fill(0);ellipse(2*x+270,2*y,floor(skip/2),floor(skip/2));      // visual marking
if (y>hy){hy=y;hyx=x;hyd=d;}          // highest point
if (y<ly){ly=y;lyx=x;lyd=d;}          // lowest most
if (x<lx){lx=x;lxy=y;lxd=d;}        // leftmost
if (x>rx){rx=x;rxy=y;rxd=d;}        // rightmost
} // eof depth range
} // eof depth mining
} // running through y
} // end of running throuhg x
//println("Highest Point:" +hy+"  Lowest Point: "+ly+"  Leftmost Point: "+lx+"  Rightmost Point: "+rx);
//// Visualisations
//fill(255,0,0);line(lx,lxy,rx,rxy);        // Left to right
println(img.height);
send();

} // end of draw

// ##########################################################################################################      
void send(){
  // closest
  OscMessage myMessageicd = new OscMessage("/closest/icd");
  myMessageicd.add(icd); /* add an int to the osc message */
  oscP5.send(myMessageicd, myRemoteLocation); /* send the message */
  OscMessage myMessage1 = new OscMessage("/closest/x");
  myMessage1.add(cdx); /* add an int to the osc message */
  oscP5.send(myMessage1, myRemoteLocation); /* send the message */
  OscMessage myMessage2 = new OscMessage("/closest/y");
  myMessage2.add(cdy); /* add an int to the osc message */
  oscP5.send(myMessage2, myRemoteLocation); /* send the message */
  OscMessage myMessage3 = new OscMessage("/closest/d");
  myMessage3.add(cd); /* add an int to the osc message */
  oscP5.send(myMessage3, myRemoteLocation); /* send the message */
  // highest
  OscMessage myMessage4 = new OscMessage("/highest/x");                  
  myMessage4.add(hyx); /* add an int to the osc message */
  oscP5.send(myMessage4, myRemoteLocation); /* send the message */
  OscMessage myMessage5 = new OscMessage("/highest/y");
  myMessage5.add(hy); /* add an int to the osc message */
  oscP5.send(myMessage5, myRemoteLocation); /* send the message */
  OscMessage myMessage6 = new OscMessage("/highest/d");
  myMessage6.add(hyd); /* add an int to the osc message */        
  oscP5.send(myMessage6, myRemoteLocation); /* send the message */
  // lowest  
  OscMessage myMessage7 = new OscMessage("/lowest/x");
  myMessage7.add(lyx); /* add an int to the osc message */
  oscP5.send(myMessage7, myRemoteLocation); /* send the message */
  OscMessage myMessage8 = new OscMessage("/lowest/y");
  myMessage8.add(ly); /* add an int to the osc message */
  oscP5.send(myMessage8, myRemoteLocation); /* send the message */
  OscMessage myMessage9 = new OscMessage("/lowest/d");
  myMessage9.add(lyd); /* add an int to the osc message */
  oscP5.send(myMessage9, myRemoteLocation); /* send the message */
  // leftest
  OscMessage myMessage10 = new OscMessage("/leftest/x");              
  myMessage10.add(lx); /* add an int to the osc message */
  oscP5.send(myMessage10, myRemoteLocation); /* send the message */
  OscMessage myMessage11 = new OscMessage("/leftest/y");
  myMessage11.add(lxy); /* add an int to the osc message */
  oscP5.send(myMessage11, myRemoteLocation); /* send the message */
  OscMessage myMessage12 = new OscMessage("/leftest/d");
  myMessage12.add(lxd); /* add an int to the osc message */
  oscP5.send(myMessage12, myRemoteLocation); /* send the message */
  // rightest
  OscMessage myMessage13 = new OscMessage("/rightest/x");            
  myMessage13.add(rx); /* add an int to the osc message */
  oscP5.send(myMessage13, myRemoteLocation); /* send the message */
  OscMessage myMessage14 = new OscMessage("/rightest/y");
  myMessage14.add(rxy); /* add an int to the osc message */
  oscP5.send(myMessage14, myRemoteLocation); /* send the message */
  OscMessage myMessage15 = new OscMessage("/rightest/d");
  myMessage15.add(rxd); /* add an int to the osc message */
  oscP5.send(myMessage15, myRemoteLocation); /* send the message */
  }
  void keyPressed()   {
  // Angle
  if (key == CODED){
  if (keyCode == UP){angle++;println("Tilt angle is now: " + angle);}
  else if (keyCode == DOWN) {angle--;println("Tilt angle is now: " + angle);}
  angle = constrain(angle, -30, 30);kinect.setTilt(angle);}
 
  
   // Max Depth
  else if (keyCode == '9') {max++;max=constrain(max, 0, 2048);println("Maximum Depth: " +max);}
  else if (keyCode == '8') {max--;max=constrain(max, 0, 2048);println("Maximum Depth: " +max);}
  else if (keyCode == '0') {max++;max=constrain(max+10, 0, 2048);println("Maximum Depth: " +max);}
  else if (keyCode == '7') {max--;max=constrain(max-10, 0, 2048);println("Maximum Depth: " +max);}
  }
  void oscEvent(OscMessage theOscMessage) {
  /* check if theOscMessage has the address pattern we are looking for. */
  if(theOscMessage.checkAddrPattern("transport/")==true) {
     /* check if the typetag is the right one. */
         if(theOscMessage.checkTypetag("i")) {
      /* parse theOscMessage and extract the values from the osc message arguments. */
    transport = theOscMessage.get(0).intValue();  
    }  
    }
     /* check if theOscMessage has the address pattern we are looking for. */
  if(theOscMessage.checkAddrPattern("tremain/")==true) {
     /* check if the typetag is the right one. */
         if(theOscMessage.checkTypetag("i")) {
      /* parse theOscMessage and extract the values from the osc message arguments. */
    tremain = theOscMessage.get(0).intValue();  
    }  
    }
         /* check if theOscMessage has the address pattern we are looking for. */
  if(theOscMessage.checkAddrPattern("bufferlength/")==true) {
     /* check if the typetag is the right one. */
         if(theOscMessage.checkTypetag("i")) {
      /* parse theOscMessage and extract the values from the osc message arguments. */
    blen = theOscMessage.get(0).intValue();  
    }  
    }
        tr = tremain/1000;
  }