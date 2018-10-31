/*
gcc lab3_01.c -o lab3
./lab3
*/

#include <stdio.h>
#include <stdlib.h>

char* two_d_alloc(int rows, int cols, int sizeOfE)
{
	char* my_array = (char*)malloc(rows*cols*sizeOfE);
	printf("Array allocated!\n");
	return my_array;
}

void two_d_dealloc(char* my_array)
{
	free(my_array);
	printf("Array deallocated!\n");
}

void two_d_store(char* my_array, int sizeOfE, int row, int col, int rows, int cols, char* value)
{
	int index = row*cols + col;
	for(int i = 0; i < sizeOfE; i++)
	{
		my_array[index*sizeOfE+i] = *(value+i);
	}
}

char* two_d_fetch(char* my_array, int sizeOfE, int row, int col, int rows, int cols)
{
	int index = row*cols + col;
	return (char*)(my_array+index*sizeOfE);
}

void memoryDump(char* my_array, int numOfBytes)
{
	int numOfWord = numOfBytes / 4;
	for(int i=0; i < numOfWord; i++)
	{
		char* my_word = my_array + i*4;
		if(i%4 == 0) printf("Address: %p -> 4 words: ", my_word);
		for(int j=3; j >= 0; j--)
		{
			char my_word = *(my_array + i*4 + j);
			// Approach1: Directly print out the word to hexadecimal,
			// However, you need to figure out how reperesent your word by 8 hexadecimal numbers. 
			//printf("%x", (unsigned char)my_word);
			
			// Approach 2: Using bitwise operations to get high four bits and low four bits
			char high_fourBits = my_word >> 4;
			high_fourBits = high_fourBits & 0x0F;
			printf("%x", (unsigned char)high_fourBits);
			char low_fourBits = my_word & 0x0F;
			printf("%x", (unsigned char)low_fourBits);
		}
		printf(" ");
		if((i+1)%4 == 0) printf("\n");
	}
}

int main()
{
	int sizeOfE;
	int rows = 12; int cols = 4;
	int row = 1; int col = 2;
	char *my_array, *fetch_value; 
	printf("Address of array before allocation %p \n", my_array);

	//------------------------Task 1: Test with integer----------------------------//
	printf("\n------Task1: Array Storage------ \n");		
	sizeOfE = sizeof(int);
	my_array = two_d_alloc(rows, cols, sizeOfE);
	printf("Address of array: %p \n", my_array);
	
	int stored_int = 256;
	two_d_store(my_array, sizeOfE, row, col, rows, cols,(char*) &stored_int);
	printf("Store integer: %d \n", stored_int);
	fetch_value = two_d_fetch(my_array, sizeOfE, row, col, rows, cols);
	int* fetch_int = (int*) fetch_value;
	printf("fetch integer: %d \n", *fetch_int);

	stored_int = -1;
	two_d_store(my_array, sizeOfE, row, col, rows, cols,(char*) &stored_int);
	printf("Store integer: %d \n", stored_int);
	fetch_value = two_d_fetch(my_array, sizeOfE, row, col, rows, cols);
	fetch_int = (int*) fetch_value;
	printf("fetch integer: %d \n", *fetch_int);

	two_d_dealloc(my_array); 

	//------------------------Task1: Test with double----------------------------//
	sizeOfE = sizeof(double); 
	my_array = two_d_alloc(rows, cols, sizeOfE);
	printf("Address of array: %p \n", my_array);

	double stored_double = 6.88;
	two_d_store(my_array, sizeOfE, row, col, rows, cols,(char*) &stored_double);
	printf("Store double: %lf \n", stored_double);
	fetch_value = two_d_fetch(my_array, sizeOfE, row, col, rows, cols);
	double* fetch_double = (double*) fetch_value;
	printf("fetch double: %lf \n", *fetch_double);
	two_d_dealloc(my_array);

	//------------------------Task2: Memory Dump----------------------------//
	printf("\n------Task2: Memory Dump------ \n");
	rows = 12; cols = 4; sizeOfE = 1; 
	//rows = 1; cols = 4; sizeOfE = 1; 	
	my_array = two_d_alloc(rows, cols, sizeOfE);
	printf("Address of array: %p \n", my_array);
	int dump_array[12]={1, 34, 63, 126, 255, 544, 1234, 12345, 26344, 54321, 123456, 32};
	
	printf("Array: ");
	for(int i=0; i<rows;i++) printf("%d ", dump_array[i]);
	printf("\n");
	two_d_store(my_array, rows*cols, 0, 0, rows, cols,(char*) dump_array);
	memoryDump(my_array, rows*cols);

	two_d_dealloc(my_array);
	return 0;
}
