// Smart Box main file

#include <Arduino.h>
#include <ArduinoJson.h>
// own files:
#include <NetworkManager.h>
#include <SensorArray.h>
#include <MainConfiguration.h>
#include <NetworkManagerStructs.h>
#include <MQTTTasks.h>

// ===================================== Global Variables =====================================
MQTTTasks *TaskMain;                                  // filled in NetworkManager.cpp, used for saving incoming Messages, FIFO order
NetworkManager *mNetwP;                               // used for usign NetworkManager access outside setup()
SensorArray *mSarrP;                                  // used for using SensorArray access outside setup()
double value_max[NUM_OF_MAXVALUES_VEHICLES_STORE];    // best optimal value from vehicle, Element 0 ist best, Element 1 is second best, etc. (decending order)
String hostname_max[NUM_OF_MAXVALUES_VEHICLES_STORE]; // name of Vehicle with best value, Element 0 ist best, Element 1 is second best, etc. (decending order)
bool hasAnswered = false;                             // variable used to see if Vehicle have answered
byte isLastRoundonError = 1;                          // currently two max values are included, if both are not responding, this Variable will be set to true, must be min 1
myJSONStr *tmp_mess;                                  // pointer to array of messages, used for iteration of messages
// -.-.-.-.-.-.-.- used for Statuses -.-.-.-.-.-.-.-
enum status_main // stores main status for Program run (main.cpp)
{
  status_isEmpty = 0,
  status_justFullPublish = 1,
  status_getOptimalVehicle = 2,
  status_hasOptVehiclePublish = 3,
  status_checkIfAckReceived = 4,
  status_checkIfTranspored = 5
};
int mcount = 0; // needed for number of messages received, lower num
int mcount2 = 0;
status_main stat = status_main::status_isEmpty;
bool toNextStatus = true;         // true if changing state, false if staying in state, it's enshuring that certain code will only run once
void (*myFuncToCall)() = nullptr; // func to call in main-loop, for finite state machine
unsigned long currentMillis = 0;  // will store current time
unsigned long previousMillis = 0; // will store last time

// -.-.-.-.-.-.-.- used for Show-Case -.-.-.-.-.-.-.-
bool showCase = true;
int waitSeconds1 = 0;  // wait between loops, without blinking LED
int waitSeconds2 = 10; // wait between loops, blinking LED
int ledState = LOW;    // used for debugging LED
/*
Notes:
  - connect only one sensor to: INPUT_PIN1
*/

// ===================================== my Function-Headers =====================================
double calcOptimum(myJSONStr &obj);
void getSmartBoxInfo();
void loopEmpty();
void publishLevel();
void getOptimalVehiclefromResponses();
void hasOptVehiclePublish();
void checkIfAckReceivedfromResponses();
void checkIfTransporedfromResponses();

// ===================================== my Helper-Functions =====================================

double calcOptimum(myJSONStr &obj) // returns Optimum for given values, higher is better, -1 if not valuable entries
{
  if (obj.vehicleParams[0] != 0 && obj.vehicleParams[1] != 0 && obj.vehicleParams[2] != 0 && obj.vehicleParams[3] != 0)
  {
    double val = 100 * (0.2 / obj.vehicleParams[0] + 0.2 / obj.vehicleParams[1] + 0.4 / obj.vehicleParams[2] + 0.2 / obj.vehicleParams[3]); // better for shorter way, 100 just for factoring
    return val;
  }
  else
  {
    LOG2("wrong parameters given to calcOptimum");
    LOG3("obj.vehicleParams[0]: " + String(obj.vehicleParams[0]) + ", obj.vehicleParams[1]: " + String(obj.vehicleParams[1]) + ", obj.vehicleParams[2]: " + String(obj.vehicleParams[2]) + ", obj.vehicleParams[3]: " + String(obj.vehicleParams[3]));
    return -1;
  }
};

void getSmartBoxInfo() // print Smart Box Information
{
  LOG2("Smart Box state: " + String(stat));
  LOG2("Hostname with best value (" + String(value_max[0]) + "): " + hostname_max[0]);
  LOG2("max Num of Vehicles in Field: " + String(NUM_OF_VEHICLES_IN_FIELD));
  LOG2("num of maxvalues to store: " + String(NUM_OF_MAXVALUES_VEHICLES_STORE));
  LOG2("Variable has answered: " + String(hasAnswered));
  LOG2("Variable isLastRoundonError: " + String(isLastRoundonError));
  LOG2("Variable mcount: " + String(mcount));
  LOG2("Variable mcount2: " + String(mcount2));
  LOG2("Variable toNextStatus: " + String(toNextStatus));
  LOG2("Time passed (currentMillis - previousMillis): " + String(currentMillis - previousMillis) + "\t\t[values: previousMillis: " + String(previousMillis) + "\tcurrentMillis: " + String(currentMillis) + "]");
};

// ===================================== my Functions =====================================
void loopEmpty() // loop until Box full
{
  digitalWrite(PIN_FOR_FULL, LOW); // if empty leave LED off or turn it off
  if (toNextStatus)                // only subscribe once but publish repeatedly
  {
    LOG1("-.-.-.- reading Sensor Values -.-.-.-");
    LOG3("entering new state: loopEmpty");
    toNextStatus = false;
  }
  if (mSarrP->getSensorData())
  {
    LOG3("is full, go next to status_justFullPublish");
    if (showCase)
      LOG2(">>>>>>>>>>>>>>>>>>>>> please now choose Option 1 in the Script (after level publish)");
    stat = status_main::status_justFullPublish;
    myFuncToCall = publishLevel;
    toNextStatus = true;
    digitalWrite(PIN_FOR_FULL, HIGH); // if full turn LED on
  }
  previousMillis = millis();
  while (((currentMillis - previousMillis) / 1000 < waitSeconds1) && (showCase))
  {
    currentMillis = millis();
  }
}

void publishLevel() // publishes SmartBox Level
{
  if (toNextStatus) // only subscribe once but publish repeatedly
  {
    LOG1("-.-.-.- pubilsh SmartBox level -.-.-.-");
    LOG3("entering new state: pubishLevel");
    mNetwP->subscribe("Vehicle/+/params");
    mNetwP->subscribe("Vehicle/+/ack");
    hasAnswered = false;
    // mcount = TaskMain->returnCurrentIterator();
    if (!TaskMain->setCurrentIteratorforIterations())
      LOG2("setting current iterator failed");
    toNextStatus = false;
    previousMillis = millis();
  }
  mNetwP->publishMessage("SmartBox/" + String(mNetwP->getHostName()) + "/level", "{\"hostname\":\"" + String(mNetwP->getHostName()) + "\",\"level\":" + String(SBLevel::full) + "}");
  mNetwP->loop();
  currentMillis = millis();
  if ((currentMillis - previousMillis) / 1000 < SMARTBOX_WAITFOR_VEHICLES_SECONDS) // wait for vehicles to send their params
  {
    currentMillis = millis();
    LOG3("now on time: " + String((currentMillis - previousMillis) / 1000) + "\t\t currentMillis: " + String(currentMillis) + "\t previousMillis: " + String(previousMillis));
    mNetwP->loop();
  }
  else if ((currentMillis - previousMillis) / 1000 >= SMARTBOX_WAITFOR_VEHICLES_SECONDS)
  {
    mNetwP->loop();
    mcount2 = TaskMain->returnCurrentIterator(); // needed for number of messages received, upper num
    tmp_mess = TaskMain->getBetween(mcount, mcount2);
    if (tmp_mess == nullptr)
    {
      LOG2("no messages");
    }
    else
    {
      LOG3("go to status_getOptimalVehicle");
      toNextStatus = true;
      stat = status_main::status_getOptimalVehicle;
      myFuncToCall = getOptimalVehiclefromResponses;
      previousMillis = millis();
      while (((currentMillis - previousMillis) / 1000 < waitSeconds1) && (showCase))
      {
        currentMillis = millis();
      }
    }
  }
  else
    LOG1("Error, time has wrong value;\tpreviousMillis: " + String(previousMillis) + "\tcurrentMillis: " + String(currentMillis));
}

void getOptimalVehiclefromResponses() // gets Vehicle with best Params due to calcOptimum(), calc Optimum Value & set hostname_max, hostname_max2
{
  if (toNextStatus) // do once
  {
    LOG1("-.-.-.- calculation of Optimal Values through Iterations -.-.-.-");
    LOG3("entering new state: getOpcimalVehiclefromResponses");
    toNextStatus = false;
    hasAnswered = false;
    mNetwP->loop();
    mcount2 = TaskMain->returnCurrentIterator(); // needed for number of messages received, upper num
    bool whileBool = true;
    // tmp_mess = TaskMain->getBetween(mcount, mcount2);
    //for (int i = 1; i < tmp_mess[0].level; i++)
    while (whileBool)
    {
      myJSONStr taskNow = TaskMain->iterateAndDoMessages();
      if (!taskNow.level == -5) // otherwise its default struct, which is not to be proceeded!
      {
        String *ttop = TaskMain->returnMQTTtopics(taskNow);
        if (ttop == nullptr)
          LOG1("no topics found, function returnMQTTtopics failed to return topics");
        // LOG3("received Topics: " + ttop[0] + ", " + ttop[1] + ", " + ttop[2]); // could be uncommented for debugging
        if ((ttop[0] == "Vehicle") && (ttop[2] == "params")) // if in MQTT topic == Vehicle/+/params
        {
          hasAnswered = true;
          LOG3("has answered, calculating Optimum for: " + ttop[1]);
          double opt = calcOptimum(taskNow);
          if (value_max[0] < opt)
          {
            value_max[1] = value_max[0];
            hostname_max[1] = hostname_max[0];
            value_max[0] = opt;
            hostname_max[0] = taskNow.hostname;
          }
          // else case not needed, two best hostnames are enough
        }
      }
      else
        whileBool = false;
    }
  }
  toNextStatus = true;
  LOG2("has calculated max Optimal Values");
  LOG3("optimal is Vehicle: " + hostname_max[0]);
  LOG3("go next to status_hasOptVehiclePublish");
  if (showCase)
    LOG2(">>>>>>>>>>>>>>>>>>>>> please now choose Option 2 in the Script (after decision getOptimalVehiclefromResponses)");
  stat = status_main::status_hasOptVehiclePublish;
  myFuncToCall = hasOptVehiclePublish;
  previousMillis = millis();
  while (((currentMillis - previousMillis) / 1000 < waitSeconds1) && (showCase))
  {
    currentMillis = millis();
  }
}

void hasOptVehiclePublish() // publishes decision for vehicle to transport Smart Box
{
  if (toNextStatus) // only subscribe once but publish repeatedly
  {
    LOG1("-.-.-.- publish decision for Vehicle -.-.-.-");
    LOG3("entering new state: hasOptVehiclePublish");
    toNextStatus = false;
    // mcount = TaskMain->returnCurrentIterator();
    if (!TaskMain->setCurrentIteratorforIterations())
      LOG2("setting current iterator failed");
    previousMillis = millis();
  }
  if (hasAnswered)
  {
    mNetwP->publishMessage("SmartBox/" + mNetwP->getHostName() + "/decision", "{\"hostname\":\"" + hostname_max[0] + "\"}"); // publishes decision, clientID is in topic
    mNetwP->loop();
    currentMillis = millis();
    if ((currentMillis - previousMillis) < SMARTBOX_ITERATION_VACKS_SECONDS) // wait for vehicles to send their acknoledgement to transport SB
    {
      LOG3("now on time: " + String(currentMillis - previousMillis));
      currentMillis = millis();
      mNetwP->loop();
    }
    else if ((currentMillis - previousMillis) >= SMARTBOX_ITERATION_VACKS_SECONDS)
    {
      mcount2 = TaskMain->returnCurrentIterator(); // needed for number of messages received, upper num
      tmp_mess = TaskMain->getBetween(mcount, mcount2);
      if (tmp_mess == nullptr)
      {
        LOG2("no messages");
        LOG3("going back to publish level!");
        stat = status_main::status_justFullPublish;
        myFuncToCall = publishLevel;
      }
      else
      {
        LOG3("go next to status_checkIfAckReceived");
        if (showCase)
          LOG2(">>>>>>>>>>>>>>>>>>>>> please now choose Option 2 in the Script (after decision publish)");
        toNextStatus = true;
        stat = status_main::status_checkIfAckReceived;
        myFuncToCall = checkIfAckReceivedfromResponses;
        if (!TaskMain->setCurrentIteratorforIterations())
          LOG2("setting current iterator failed");
        // mcount = TaskMain->returnCurrentIterator();
        mNetwP->loop();
        previousMillis = millis();
        while (((currentMillis - previousMillis) / 1000 < waitSeconds1) && (showCase))
        {
          currentMillis = millis();
        }
      }
    }
    else
      LOG1("Error, time has wrong value;\tpreviousMillis: " + String(previousMillis) + "\tcurrentMillis: " + String(currentMillis));
  }
  else
  {
    LOG2("unfortunately no answeres arrived");
    stat = status_main::status_justFullPublish;
    myFuncToCall = publishLevel;
  }
}

void checkIfAckReceivedfromResponses() // runs until acknoledgement of desired Vehicle arrived, check if right Vehicle answered to get SmartBox transported
{
  mNetwP->loop();
  mcount2 = TaskMain->returnCurrentIterator();
  if (toNextStatus) // only subscribe once but publish repeatedly
  {
    LOG1("-.-.-.- check for transport acknoledgement -.-.-.-");
    LOG3("entering new state: checkIfAckReceivedfromResponses");
    hasAnswered = false;
    toNextStatus = false;
    isLastRoundonError = 0;
  }
  tmp_mess = TaskMain->getBetween(mcount, mcount2);
  if (tmp_mess == nullptr)
  {
    LOG2("no messages for transport acknoledgement received yet");
    //stat = status_main::status_justFullPublish; // jump back to publish
    //myFuncToCall = publishLevel;
  }
  else
  {
    bool whileBool = true;
    //for (int i = 1; i < tmp_mess[0].level; i++)
    while (whileBool)
    {
      myJSONStr taskNow = TaskMain->iterateAndDoMessages();
      if (!taskNow.level == -5) // otherwise its default struct, which is not to be proceeded!
      {
        String *ttop = TaskMain->returnMQTTtopics(taskNow);
        if (ttop == nullptr)
          LOG1("no topics found, function returnMQTTtopics failed to return topics");
        // LOG3("ttop[0]: " + ttop[0] + "\t ttop[1]: " + ttop[1] + "\t ttop[2]: " + ttop[2] + "\t taskNow.hostname: " + taskNow.hostname);                        // could be uncommented for debugging
        if ((ttop[0] == "Vehicle") && (ttop[1] == hostname_max[0]) && (ttop[2] == "ack") && (hasAnswered == false) && (taskNow.hostname == mNetwP->getHostName())) // if desired Vehicle answered
        {
          hasAnswered = true;
          LOG3("right Vehicle has answered and send acknoledgement to transport it");
        }
      }
      else
        whileBool = false;
    }
    if (hasAnswered) // if right Vehicle answered, go next
    {
      LOG3("go forward to status_checkIfTranspored");
      if (showCase)
        LOG2(">>>>>>>>>>>>>>>>>>>>> please now choose Option 3 in the Script");
      toNextStatus = true;
      stat = status_main::status_checkIfTranspored;
      myFuncToCall = checkIfTransporedfromResponses;
    }
    else if (isLastRoundonError <= NUM_OF_VEHICLES_IN_FIELD)
    {
      LOG3("go next to status_hasOptVehiclePublish, reset hostname");
      toNextStatus = true;
      stat = status_main::status_hasOptVehiclePublish;
      myFuncToCall = hasOptVehiclePublish;
      hostname_max[0] = hostname_max[isLastRoundonError];
    }
    else
    {
      LOG1("none of the two desired vehicles ansered");
      toNextStatus = true;
      stat = status_main::status_justFullPublish;
      myFuncToCall = publishLevel;
    }
  }
  //mcount = mcount2;
  int timeIt = 0;
  previousMillis = millis();
  previousMillis = millis();
  while (((currentMillis - previousMillis) / 1000 < waitSeconds1) && (showCase))
  {
    currentMillis = millis();
  }
}

void checkIfTransporedfromResponses() // runs until SmartBox is transpored, emtied and brought back to factory
{
  if (toNextStatus) // only subscribe once but publish repeatedly
  {
    LOG1("-.-.-.- check if SmartBox is transported -.-.-.-");
    LOG3("entering new state: checkIfTransporedfromResponses");
    toNextStatus = false;
    hasAnswered = false;
    if (!TaskMain->setCurrentIteratorforIterations())
      LOG2("setting current iterator failed");
    // mcount = TaskMain->returnCurrentIterator();
  }
  mNetwP->loop();
  mcount2 = TaskMain->returnCurrentIterator();
  tmp_mess = TaskMain->getBetween(mcount, mcount2);
  // LOG3("size of array: " + String(tmp_mess[0].level)); // could be used for debugging
  if (tmp_mess == nullptr)
  {
    LOG2("no messages");
  }
  else
  {
    LOG3("now im here");
    bool whileBool = true;
    //for (int i = 1; i < tmp_mess[0].level; i++)
    while (whileBool)
    {
      myJSONStr taskNow = TaskMain->iterateAndDoMessages();
      if (!taskNow.level == -5) // otherwise its default struct, which is not to be proceeded!
      {
        String *ttop = TaskMain->returnMQTTtopics(taskNow);
        if (ttop == nullptr)
          LOG1("no topics found, function returnMQTTtopics failed to return topics");
        // LOG3("ttop[0]: " + ttop[0] + "\t ttop[1]: " + ttop[1] + "\t ttop[2]: " + ttop[2] + "\t taskNow.request:" + taskNow.request);                          // could be uncommented for debugging
        if ((ttop[0] == "Vehicle") && (ttop[1] == hostname_max[0]) && (ttop[2] == "ack") && (hasAnswered == false) && (taskNow.request == mNetwP->getHostName())) // if desired Vehicle answered
        {
          hasAnswered = true;
          LOG2("right Vehicle has answered and sent transported acknoledgement");
        }
        else
          LOG3("not the right answer");
      }
      else
        whileBool = false;
    }
    if (hasAnswered) // if Vehicle is transported, since transported and brought back to factory unsubsribe (is empty again)
    {
      LOG3("go next to status_isEmpty");
      toNextStatus = true;
      mNetwP->unsubscribe("Vehicle/+/params"); // when transported and brought back to factory
      mNetwP->unsubscribe("Vehicle/+/ack");
      stat = status_main::status_isEmpty;
      myFuncToCall = loopEmpty;
    }
  }
  //mcount = TaskMain->returnCurrentIterator();
  previousMillis = millis();
  while (((currentMillis - previousMillis) / 1000 < waitSeconds1) && (showCase))
  {
    currentMillis = millis();
  }
}

// ===================================== Arduino Functions =====================================
void setup() // for initialisation
{
  if (LOGLEVELCONFIGURATION > 0)
  {
    Serial.begin(12000); //Initialize serial
    while (!Serial)
      ; // wait for serial port to connect
  }
  LOG1("-.-.-.- SETUP -.-.-.-");
  LOG3("entering setup - initializing components");
  mNetwP = new NetworkManager();
  mSarrP = new SensorArray();
  TaskMain = mNetwP->NetManTask_classPointer;
  pinMode(PIN_FOR_FULL, OUTPUT);
  myFuncToCall = loopEmpty;

  if (showCase)
  {
    pinMode(13, OUTPUT); // debug LED
    // mNetwP->subscribe("SmartBox/+/level");
    // mNetwP->subscribe("Vehicle/+/params");
    // mNetwP->subscribe("SmartBox/+/params");
  }
}

void loop() // one loop per one cycle (SB full -> transported -> returned empty)
{
  static int i;
  // TaskMain->printAllMessages(0); // to show all saved Tasks (hostnames)

  if (showCase)
  {
    // mNetwP->subscribe("hello");
    // mNetwP->publishMessage("hello", "{hostname:heyhey-" + String(i) + "}");

    int timeIt = 0;
    int previousMilliss = millis();
    while ((timeIt < waitSeconds2) && (showCase)) // wait before next loop
    {
      if (currentMillis - previousMilliss >= 500)
      {
        timeIt++;
        previousMilliss = currentMillis;
        if (ledState == LOW)
        {
          ledState = HIGH;
        }
        else
        {
          ledState = LOW;
        }
        digitalWrite(13, ledState);
        mNetwP->loop(); // needed to be called regularly to keep connection alive
      }
      currentMillis = millis();
    }

    LOG1();
    LOG1();
    LOG1();
    LOG1("-------------------------- now going to loop again: " + String(i) + "-------------------------- ");
    i++;
    LOG2("STATE: " + String(stat));
    LOG2("--------------------------");
    myFuncToCall();
  }
  else
  {
    LOG1("-------------------------- now going to loop again: " + String(i) + "-------------------------- ");
    i++;
    LOG2("STATE: " + String(stat));
    LOG2("--------------------------");
    myFuncToCall();
    mNetwP->loop(); // needed to be called regularly to keep connection alive
  }
}

/*
Show Case:
commands on Raspb
mosquitto_pub -t Vehicle/Vehicle1234/params -m "{hostname:Vehicle1234,	params:[1,1,1,1]}"
mosquitto_pub -t Vehicle/Vehicle1234/ack -m "{hostname:SmartBox1234}"
mosquitto_pub -t Vehicle/Vehicle1234/ack -m "{request:SmartBox1234}"
*/