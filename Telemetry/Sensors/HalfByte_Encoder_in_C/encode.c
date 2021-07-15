/*
 ============================================================================
 Name        : encode.c
 Author      : Daniel Reimer
 Version     :
 Description : Proof of Concept for character encoder
 Encodes two characters (because only integers, commas, and periods are needed)
 as one and then decodes them
 ============================================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define FALSE 0
#define TRUE !(FALSE)

int combine(unsigned char *total, float accel_x, float accel_y, float accel_z, float pitch, float roll, float yaw, float altitude, double lat, double longg, char* time);
int encode(unsigned char *buffer, float accel_x, float accel_y, float accel_z, float pitch, float roll, float yaw, float altitude, double lat, double longg, char* time);
int decode(int encodedlen, unsigned char *encoded, unsigned char *buffer);
void strrev(char *str);

int main(void) {

	//large buffer of arbitrary length
	//(has to be large enough to contain input data)
	unsigned char total[1000];

	//data to encode
	//double accel = 12345;
	//double accel2 = 6789;

	float accel_x = 1.0032;
	float accel_y = 0.13214;
	float accel_z = 0.045221;

	float pitch = 12.452;
	float roll = 47.00;
	float yaw = 2.3442;
	float altitude = 1536.3;

	double lat = 12253356;
	double longg = 53253356;

	char time[] = "23:04:30";




	//the encode function returns the
	//length of the encoded character array
	int encodedlen = encode(total, accel_x, accel_y, accel_z, pitch, roll, yaw, altitude, lat, longg, time);
	//total now points to the encoded character array


	printf("encoded length: %d\n", encodedlen);
	printf("encoded data: %s\n", total);



	printf("\n");
	//BELOW THIS PART IN MAIN MIMICS THE RECEIVING SIDE
	//all that is accessible to the receiver is the encoded character array and its length
	printf("\n");


	//encoded pointer points to the encoded array
	//this will make the rest of the code easier to follow
	unsigned char *encoded = total;

	//create an array to store the decoded data
	//it will be twice the length of the encoded array
	//but we will just make it 1000
	unsigned char *decoded[1000];


	fflush( stdout);



	//decodes the encoded array and stores it in the decoded array
	decode(encodedlen, encoded, decoded);


	//can use strlen function now because everything in
	//the decoded array should be ASCII characters 0-9 or '.' or ';'
	int decodedlen = strlen(decoded);


	printf("decoded length: %d\n", decodedlen);
	printf("decoded data: %s\n", decoded);

	fflush(stdout);

}










/**
 *encode - the pointer to the blank character array (buffer)
 *will point to the encoded character array
 *
 *
 *
 *@param *buffer - a pointer to a blank array which will point to the encoded array
 *@param accel - a piece of acceleration data
 *@param accel2 - another piece of acceleration data
 *
 *@return encodedlen - the length of the encoded character array
 */
int encode(unsigned char *buffer, float accel_x, float accel_y, float accel_z, float pitch, float roll, float yaw, float altitude, double lat, double longg, char *time) {

	//combine all inputs into one character array
	combine(buffer, accel_x, accel_y, accel_z, pitch, roll, yaw, altitude, lat, longg, time);

	//declare variables
	int encodedlen;
	int bufferlen = strlen(buffer);

	//this project assumes that the decoded character array
	//will always contain an even number of characters
	//the encoded character array will be half
	//the length of the decoded character array
	encodedlen = (bufferlen / 2);

	//iterate through buffer and encode two characters at a time
	//store encoded characters back in buffer
	int j = 0;

	for (int i = 0; i < bufferlen; i += 2) {

		//subtract '0' so that each character can be represented by four bits
		buffer[i] = buffer[i] - '0';
		buffer[i + 1] = buffer[i + 1] - '0';

		//bit shift the first character
		buffer[i] = buffer[i] << 4;

		//add two characters together and store back in buffer
		buffer[j] = buffer[i] | buffer[i + 1];

		j++;
	}

	//end encoded character array at the end of the encoded characters
	buffer[j] = '\0';


	//the first half of buffer will contain the encoded characters
	return encodedlen;

}



















/**
 *
 *combine - combines all data into one long character array and makes sure everything is
 *0-9 or ; (to separate inputs) or < (to replace periods)--in ASCII periods are a lower
 *index than integers so if encode wants to subtract by '0' there can be no characters with a lower index than '0'
 *
 *
 *@param *total - a pointer to a blank array which will point to the combined array
 *@param accel - a piece of acceleration data
 *@param accel2 - another piece of acceleration data
 *
 *@return 0 - if the function completes
 */
int combine(unsigned char *total, float accel_x, float accel_y, float accel_z, float pitch, float roll, float yaw, float altitude, double lat, double longg, char *time) {

	int firstnum = FALSE;
	int start = FALSE;

	//initialize a temporary array with all zeros
	unsigned char temp[1000] = { '0' };
	for (int i = 0; i < 1000; i++) {
		temp[i] = '0';

	}

	//print everything to long total char array
	sprintf(temp, "%s", time);
	sprintf(temp + sizeof(time), "%c", ';');
	sprintf(temp + sizeof(time) + sizeof(unsigned char), "%f", accel_x);
	sprintf(temp + sizeof(time) + sizeof(unsigned char) + sizeof(double), "%c", ';');
	sprintf(temp + sizeof(time) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char), "%f", accel_y);
	sprintf(temp + sizeof(time) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char) + sizeof(double), "%c", ';');
	sprintf(temp + sizeof(time) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char), "%f", accel_z);
	sprintf(temp + sizeof(time) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char) + sizeof(double), "%c", ';');
	sprintf(temp + sizeof(time) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char), "%f", pitch);
	sprintf(temp + sizeof(time) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char) + sizeof(double), "%c", ';');
	sprintf(temp + sizeof(time) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char), "%f", roll);
	sprintf(temp + sizeof(time) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char) + sizeof(double), "%c", ';');
	sprintf(temp + sizeof(time) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char), "%f", yaw);
	sprintf(temp + sizeof(time) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char) + sizeof(double) , "%c", ';');
	sprintf(temp + sizeof(time) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char), "%f", altitude);
	sprintf(temp + sizeof(time) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char)+ sizeof(double), "%c", ';');
	sprintf(temp + sizeof(time) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char)+ sizeof(double) + sizeof(unsigned char), "%f", lat);
	sprintf(temp + sizeof(time) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char)+ sizeof(double) + sizeof(unsigned char) + sizeof(double), "%c", ';');
	sprintf(temp + sizeof(time) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char)+ sizeof(double) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char), "%f", longg);
	sprintf(temp + sizeof(time) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char)+ sizeof(double) + sizeof(unsigned char) + sizeof(double) + sizeof(unsigned char) + sizeof(double), "%c", ';');


	//get rid of extra zeros that were added when using the sprintf function
	//ends at 9 because there is a separate for loop just for time
	int j = 0;
	for (int i = 999; i >= 9; i--) {

		if (temp[i] != '0' && temp[i] != '.' && start) {
			firstnum = TRUE;
		}

		if (temp[i] == ';') {
			start = TRUE;
			firstnum = FALSE;
		}

		if (temp[i] == '.' && firstnum) {
			total[j] = temp[i];
			j++;

		}

		if (temp[i] != '0' && start && temp[i] != '.') {

			total[j] = temp[i];
			j++;

		}

		if (temp[i] == '0' && firstnum) {
			total[j] = temp[i];
			j++;


		}

	}


	//extra for loop just for the time
	for(int i = 8; i >=0; i--){
		total[j] = temp[i];
		j++;
	}



	//reverse total
	strrev(total);

	//makes sure the output has an even number of characters
	//if total has an odd number of characters the last comma is removed
	if (strlen(total) % 2 != 0) {
		total[strlen(total) - 1] = '\0';
	}

	int totallen = strlen(total);

	//replace periods with <
	for (int i = 0; i < totallen; i++) {
		if (total[i] == '.') {
			total[i] = '<';
		}
	}

	printf("combined data: %s\n", total);

	return 0;
}



















/**
 *
 *decode - decodes the encoded character array, and writes
 *the decoded characters to the blank buffer array
 *
 *
 *@param encodedlen - the length of the encoded array
 *@param *encoded - a pointer to the encoded character array
 *@param *buffer - a pointer to a blank character array,
 *					which the decoded data will be written to
 *
 *@return 0 - if the function completes
 */
int decode(int encodedlen, unsigned char *encoded, unsigned char *buffer) {


	//iterate through the encoded character array for
	//twice the encoded array length
	int j = 0;
	int i = 0;
	while (i < 2*encodedlen) {

		//mask the first and second characters
		buffer[i + 1] = encoded[j] & 0xF;
		buffer[i] = encoded[j] & 0xF0;

		//bit shift the first character
		buffer[i] = buffer[i] >> 4;

		//add '0' back to the characters
		buffer[i] = buffer[i] + '0';
		buffer[i + 1] = buffer[i + 1] + '0';

		//replace '<' with '.'
		if (buffer[i] == '<') {
			buffer[i] = '.';
		}

		if (buffer[i + 1] == '<') {
			buffer[i + 1] = '.';
		}

		j++;
		i+=2;

	}

	buffer[i] = '\0';

	return 0;
}










/**
 *
 *reverses the input character array
 *
 *
 *@param *str - a pointer to the character array to be reversed
 */
void strrev(char *str) {
			int len = strlen(str);
			char tmp;
			int i;
			for (i=0; i<len/2; i++) {
				tmp = str[i];
				str[i] = str[len-i-1];
				str[len-i-1]=tmp;
			}
}
