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

int main()
{
	int sizeOfE;
	int rows = 6; int cols = 8;
	int row = 3; int col = 4;
	char *my_array, *fetch_value; 
	printf("Address of array before allocation %p \n", my_array);

	//------------------------Test with integer----------------------------//
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

	//------------------------Test with double----------------------------//
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
		        
	return 0;
}
