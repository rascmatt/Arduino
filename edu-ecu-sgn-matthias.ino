int sensorPin = A0;
int sensorValue = 0;
int poti_pin1 = A1;
int poti_pin2 = A2;

// must be long for calculation process
long poti_value = 0;
long value = 0;
long value_add = 0;
bool ausfall = false;

const int outPin = 9;
const int injconfPin = 8; // injection confirmation signal out
const int threshold = 15;//32; // 0...0V 1023...5V
const int gap_length = 3; //overall gap length of the NE input signal in units of t_2

// floats bis zur 4. Nachkommastelle praezise at t of NE signal=8ms (block_length approx. 10800*10µs)
const float del_start = 0.323;// in multiples of block length. delay between the first NE pulse of a ten pulse row and the spv closing.
const float del_length = 0; //operation width of the poti, in multiples of t_2.

const float spv_cl_add = 0; // in multiples of block length. APPROX time the spill valve is closed that adds to poti range
const float spv_cl_range = 0.48;//31-del_start-spv_cl_add; // poti range. in multiples of t_2
const float faktor = 0.84; //to detect every gap, spill valve operating must be shorter than the gap to gap time shortened by another t_2 as a buffer

unsigned long ms, ms1, ms2, ms3, ms4, ms_block; ///###### unsigned long - also do in former versions
int i=0;// overflow does not matter

void setup() {
  pinMode(outPin, OUTPUT);
  pinMode(injconfPin, OUTPUT);
  ms1 = 0;
  ms2 = 0;
  digitalWrite(outPin, LOW);
  digitalWrite(injconfPin, HIGH);
  ms3=0;
  Serial.begin(9600);
}


bool previousPhase; //set true if high && set false if low
bool currentPhase

unsigned long prevT, currT, t1, t2

bool c;

void loop() {
  prevT = 0;
  currT = 0;
  currentPhase = true;
  c = false;

  do {

    //makes the loop continue the first 2 iterations
    c = prevT==0; //true if prevT==0 else false

    //update previous time & phase
    prevT = currT;
    previousPhase = currentPhase;

    //measur time of current phase
    sensorValue = analogRead(sensorPin);
    t1 = micros();
    if(sensorValue >= threshold){
      while(sensorValue >= threshold){
        sensorValue = analogRead(sensorPin);
      }
      currentPhase = true;
    }else{
      while(sensorValue < threshold){
        sensorValue = analogRead(sensorPin);
      }
      currentPhase = false;
    }
    t2 = micros();

    currT = t2-t1;

    //TODO: detect loss of signal

  }while( !currentPhase && (prevT*1.5 > currT) || c);

  // nach Signalverlust wieder hinauf zur gap-detektierenden Schleife
  if ( ms3==0 or ausfall){
    ausfall = false;
    ms3 = micros();
  } else {
    value = 0;
    value_add = 0;
    ms4 = micros();
    ms_block = round((ms4 - ms3)*faktor);
    ms3 = ms4;

     // do not run the following in the first runthrough (ms3=0) as there is no block length available yet
    // do also not run if signal was lost (below threshold) for some time. In this case, it must not be run for a second time (block_again).
    poti_value = analogRead(poti_pin1);
    value = round(poti_value*ms_block*del_length/10230); //KEEP the divisor at the last position //*10µs <16000, so suitable for delayMicroseconds
    value_add = round(del_start*ms_block/10);

    for (int i=0; i < 10; i=i+1) { // int i=0 ######## - also do in former versions
      delayMicroseconds(value_add);
      delayMicroseconds(value);
    }

    digitalWrite(outPin, HIGH);
    digitalWrite(injconfPin, LOW);

    poti_value = analogRead(poti_pin2);
    value = round(poti_value*ms_block*spv_cl_range/10230); //KEEP the divisor at the last position
    value_add = round(spv_cl_add*ms_block/10);

    for (int i=0; i < 10; i=i+1) {
      delayMicroseconds(value_add);
      delayMicroseconds(value);
    }

    digitalWrite(outPin, LOW);
    digitalWrite(injconfPin, HIGH);
  }
}
