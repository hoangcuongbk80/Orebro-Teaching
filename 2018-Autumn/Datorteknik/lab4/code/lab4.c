#include <stdio.h>
#include <stdlib.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#define bufferSize 16

typedef struct
{
	char *m_buffer;
	char position;
	char length;
}buffer;

buffer* setupBuffer(char size)
{
	buffer* new_buffer;
	new_buffer = malloc(sizeof(buffer));
	new_buffer->m_buffer = malloc(size);
	new_buffer->position = bufferSize;
	new_buffer->length = bufferSize;
	return new_buffer;
}

char buf_in(buffer* b, int byteFile)
{
	if((*b).position >= (*b).length)
	{
		int t = read(byteFile, b->m_buffer, b->length);
		(*b).position = 0;
		printf("Filled buffer: %s \n", b->m_buffer);
	}
	char byte = *(b->m_buffer + b->position);
	b->position++;
	return byte;
}

void buf_out(buffer* b, int byteFile, char byte)
{
	*(b->m_buffer + b->position) = byte;
	if((*b).position >= (*b).length)
	{
		int t = write(byteFile, b->m_buffer, b->length);
		b->position = 0;
	}
	b->position++;
}

void buf_flush(buffer* b, int byteFile)
{

}

int main()
{
	buffer* b_int = setupBuffer(bufferSize);
	buffer* b_out = setupBuffer(bufferSize);
	b_out->position = 0;

	FILE* writeFile = fopen("buf_in.bin", "wb");

	char x[32]="AASS-OREBRO-UNIVERSITY-SWEDEN-VN";
	fwrite(x, sizeof(x[0]), sizeof(x)/sizeof(x[0]), writeFile);
	fclose(writeFile);

	int openFile = open("buf_in.bin", O_RDONLY | O_APPEND);
	int outFile = open("buf_out.txt", O_WRONLY | O_CREAT);
	int i;
	for(i=0; i < 32; i++)
	{
		if(i%4 == 0)
		{
			printf("\n");
		}
		char ch = buf_in(b_int, openFile);
		printf("%c ", ch);
		buf_out(b_out, outFile, ch);
	}
	/*buf_flush(b_out, outFile);*/
	printf("\n");
	free(b_int->m_buffer);
	free(b_out->m_buffer);

	return 0;
}
