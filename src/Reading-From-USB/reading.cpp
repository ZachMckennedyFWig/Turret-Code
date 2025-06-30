#include "reading.h"
#include "vex.h"

#include "../RobotParameters/parameters.h"
#include "../Structures/structures.h"
#include "../General/general.h"

using namespace std;
using namespace vex;

/*
Functions required to read the target info from the raspberry pi. 

You don't actually need all the char to string conversion code but I thought it was causing a problem so I wrote it that way and just never changed it back. 
*/


// Function to combine all members of a character array and output a string
string combine_char(char words[])
{
	// String to store the temporary string made from combining the words
	string temp = "";

	// Loop all values in array
	for(int i = 0; i < buffer_size; i++)
	{
		// Combines the characters together to make string
		temp = temp + words[i];
	}
	return temp;
}


// Task to keep reading the serial input to the v5 brain
int Read_Loop()
{
	// Array to store the recieved serial input
	char buffer[buffer_size];

	// String to store the serial input temporarily
	string temp = "";

	while(true)
	{
		// Read the given length of serial input from stdin (Which is the location
		// of the serial input) and store it in the buffer array
		//
		// *NOTE: fread will wait until it gets a serial input that is BUFFER_SIZE
		// long. Nothing will progress past this line until it gets something that
		// size.
		fread(buffer, 1,buffer_size,stdin);

		// Combine the buffer into a single string
		temp = combine_char(buffer);

    //Brain.Screen.printAt(0,160, "%d", buffer[0]- 48);

    if(buffer[0] - 48 == 0)
    {cam.y = (buffer[1] - 48) * 10 + (buffer[2] - 48); }
    else 
    {cam.y = (buffer[0] - 48) * 100 + (buffer[1] - 48) * 10 + (buffer[2] - 48); }

    if(buffer[3] - 48 == 0)
    {cam.x = (buffer[4] - 48) * 10 + (buffer[5] - 48); }
    else 
    {cam.x = (buffer[3] - 48) * 100 + (buffer[4] - 48) * 10 + (buffer[5] - 48); }

    // Prints coords on the brain screen
    print_coords(cam.x, cam.y);

		// Don't hog CPU >:(
		wait(10, msec);
  }
  return 0;
}