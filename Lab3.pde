/* library imports *****************************************************************************************************/ 
import processing.serial.*;
import static java.util.concurrent.TimeUnit.*;
import java.util.concurrent.*;
/* end library imports *************************************************************************************************/  



/* scheduler definition ************************************************************************************************/ 
private final ScheduledExecutorService scheduler      = Executors.newScheduledThreadPool(1);
/* end scheduler definition ********************************************************************************************/ 



/* device block definitions ********************************************************************************************/
Board             haplyBoard;
Device            widgetOne;
Mechanisms        pantograph;

byte              widgetOneID                         = 5;
int               CW                                  = 0;
int               CCW                                 = 1;
boolean           renderingForce                      = false;
/* end device block definition *****************************************************************************************/



/* framerate definition ************************************************************************************************/
long              baseFrameRate                       = 120;
/* end framerate definition ********************************************************************************************/ 



/* elements definition *************************************************************************************************/

/* Screen and world setup parameters */
float             pixelsPerCentimeter                 = 40.0;

/* generic data for a 2DOF device */
/* joint space */
PVector           angles                              = new PVector(0, 0);
PVector           torques                             = new PVector(0, 0);

/* task space */
PVector           posEE                               = new PVector(0, 0);
PVector           fEE                                 = new PVector(0, 0); 

/* World boundaries in centimeters */
FWorld            world;
float             worldWidth                          = 25.0;  
float             worldHeight                         = 10.0; 

float             edgeTopLeftX                        = 0.0; 
float             edgeTopLeftY                        = 0.0; 
float             edgeBottomRightX                    = worldWidth; 
float             edgeBottomRightY                    = worldHeight;


/* Initialization of interative shapes */
FBox              b;
FPoly             t;
FCircle           g;
FCircle           e;
FBlob             f;


/* Initialization of virtual tool */
HVirtualCoupling  s;
PImage            haplyAvatar;

/* end elements definition *********************************************************************************************/ 

int simulation = 0;   // 0 = water and air, 1 = earth



/* setup section *******************************************************************************************************/
void setup(){
  /* put setup code here, run once: */
  
  /* screen size definition */
  size(1000, 400);
  
  /* device setup */
  
  /**  
   * The board declaration needs to be changed depending on which USB serial port the Haply board is connected.
   * In the base example, a connection is setup to the first detected serial device, this parameter can be changed
   * to explicitly state the serial port will look like the following for different OS:
   *
   *      windows:      haplyBoard = new Board(this, "COM10", 0);
   *      linux:        haplyBoard = new Board(this, "/dev/ttyUSB0", 0);
   *      mac:          haplyBoard = new Board(this, "/dev/cu.usbmodem1411", 0);
   */ 
  haplyBoard          = new Board(this, Serial.list()[2], 0);
  widgetOne           = new Device(widgetOneID, haplyBoard);
  pantograph          = new Pantograph();
  
  widgetOne.set_mechanism(pantograph);

  widgetOne.add_actuator(1, CCW, 2);
  widgetOne.add_actuator(2, CW, 1);
 
  widgetOne.add_encoder(1, CCW, 241, 10752, 2);
  widgetOne.add_encoder(2, CW, -61, 10752, 1);
    
  widgetOne.device_set_parameters();
  
  
  /* 2D physics scaling and world creation */
  hAPI_Fisica.init(this); 
  hAPI_Fisica.setScale(pixelsPerCentimeter); 
  world               = new FWorld();
  
  
  
  /* Haptic Tool Initialization */
  s                   = new HVirtualCoupling((1)); 
  s.h_avatar.setDensity(4); 
  s.init(world, edgeTopLeftX+worldWidth/2, edgeTopLeftY+2); 
  s.h_avatar.setFillColor(color(0,0,0));

  /* world conditions setup */
  world.setGravity((0.0), (1000.0)); //1000 cm/(s^2)
  //world.setGravity((0.0), (0.0));
  world.setEdges((edgeTopLeftX), (edgeTopLeftY), (edgeBottomRightX), (edgeBottomRightY)); 
  world.setEdgesRestitution(.4);
  world.setEdgesFriction(0.5);
  
  
  world.draw();
  
  
  /* setup framerate speed */
  frameRate(baseFrameRate);
  

  /* setup simulation thread to run at 1kHz */ 
  SimulationThread st = new SimulationThread();
  scheduler.scheduleAtFixedRate(st, 1, 1, MILLISECONDS);
}
/* end setup section ***************************************************************************************************/

void keyPressed() {
  if (key == ' ') { // pressing spacebar makes walls flexible
    if (simulation == 0){
      simulation = 1;
    }else if (simulation == 1){
      simulation = 0;
    }
  }
  println("SPACE BAR");
}


/* draw section ********************************************************************************************************/
void draw(){
  /* put graphical code here, runs repeatedly at defined framerate in setup, else default at 60fps: */
  if(renderingForce == false){
    background(255);
    world.draw();
  }
}
/* end draw section ****************************************************************************************************/



/* simulation section **************************************************************************************************/
class SimulationThread implements Runnable{
  
  public void run(){
    /* put haptic simulation code here, runs repeatedly at 1kHz as defined in setup */
    
    renderingForce = true;
    
    if(haplyBoard.data_available()){
      /* GET END-EFFECTOR STATE (TASK SPACE) */
      widgetOne.device_read_data();
    
      angles.set(widgetOne.get_device_angles()); 
      posEE.set(widgetOne.get_device_position(angles.array()));
      posEE.set(posEE.copy().mult(200));  
    }
    
    s.setToolPosition(edgeTopLeftX+worldWidth/2-(posEE).x, edgeTopLeftY+(posEE).y-7); 
    
    if(simulation == 0){
      if(s.h_avatar.getY() <= 4.5){
        // ==== AIR ====
        s.h_avatar.setDamping(0);
        
        s.updateCouplingForce();
        fEE.div(100000); //dynes to newtons

        fEE.x = -s.h_avatar.getVelocityX()/70;
        fEE.y = -s.h_avatar.getVelocityY()/70;

      }else{
        // ==== WATER ====
        
        // Compute "depth" based damping.
        float damping = 800 + s.h_avatar.getY()*15.0 + constrain((-s.h_avatar.getY() + 5)*150, 0, 99999);
        s.h_avatar.setDamping(damping);
        s.updateCouplingForce();
        fEE.set(-s.getVirtualCouplingForceX(), s.getVirtualCouplingForceY());
        fEE.div(100000); //dynes to newtons

      }
    }else if (simulation == 1){
      // ==== EARTH ====
      s.h_avatar.setDamping(0);
      s.updateCouplingForce();
      fEE.div(100000); //dynes to newtonss


      // If the magnitude of movement is small enough, ignore it
      float magnitude = sqrt(pow(s.h_avatar.getVelocityX(), 2) + pow(s.h_avatar.getVelocityY(), 2));
      if(magnitude < 0.1){
        magnitude = 0;
      }
      
      // scalar for the haptic effect
      float multiplier = 20;
      println(magnitude/multiplier * random(-1, 1));

      float xMag = random(-1, 1);
      float yMag = random(-1, 1);
      
      // compute the forces
      float xForce = magnitude * xMag/(sqrt(pow(xMag, 2) + pow(yMag, 2))) / multiplier;
      float yForce = magnitude * yMag/(sqrt(pow(xMag, 2) + pow(yMag, 2))) / multiplier;
      
      fEE.x = constrain(xForce, -1.5, 1.5);
      fEE.y = constrain(yForce, -1.5, 1.5);
      
      
    }
    
    
    torques.set(widgetOne.set_device_torques(fEE.array()));
    widgetOne.device_write_torques();
  
    world.step(1.0f/1000.0f);
  
    renderingForce = false;
  }
}
/* end simulation section **********************************************************************************************/



/* helper functions section, place helper functions here ***************************************************************/

/* end helper functions section ****************************************************************************************/
