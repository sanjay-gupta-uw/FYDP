/*
The following code is the skeleton code for the coil winding machine. It merely highlights points of 
interest that of importance, as well as defining the functions that will be used in theory to meet
project specifications
*/

// define constants, variables
int WINDING_ID;
int GAUGE;
int TOTAL_WINDINGS;

int servoAdd = 1;
int servoPos = 90;   // start position
// length/height being wrapped?

// define functions
int computeWireDistance(); // determine how far apart the windings should be
void performWind();  // rotate motor once
void nextWirePlacement(); // configure rotation to match calculation for distance
void reset();  // reloads the program so inputs can be retyped
void updateInterface(); // specific to LCD display

void setup()
{
    Serial.begin(9600);  // computer printing interface only
    // initialize pins

    // setup LCD, get input for 
}

void loop()
{
    for (int i = 0; i < TOTAL_WINDINGS; ++i)
    {
        performWind();
        nextWirePlacement();
        updateInterface();
    }
    Serial.println("Windings complete"); // testing only, displays in serial monitor in computer
}Q