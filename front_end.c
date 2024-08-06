#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "polygon.h"
#include "front_end.h"

static void echo(int value);

#define ON 1
#define OFF 0

static FILE *out_file;
static FILE *in_file;
static char out_fileHdr[255];
static char out_filename[80];
static char in_filename[80];

static f_point pts[] = {0.0, 0.0,
						3.0, 0.0,
						3.0, 1.0,
						2.0, 1.0,
						2.0, 2.0,
						3.0, 2.0,
						3.0, 3.0,
						0.0, 3.0,
						0.0, 2.0,
						1.0, 2.0,
						1.0, 1.0,
						0.0, 1.0,
						-1.0, -1.0};

int main()
{

	PolyHndl thePolygon = NULL;
	int prev, next, N, i;
	f_type area, angle1, angle2;

	echo(ON);

	in_file = get_in_file(in_filename);
	out_file = get_out_file(out_filename, out_fileHdr);

	if (*out_fileHdr)
	{
		myfprintf(out_file, "\n %s \n", out_fileHdr);
	}

	thePolygon = newPolygon();

	for (i = 0, N = 1; (pts[i].x >= 0.0) && N; i++)
	{
		N = addVertex(thePolygon, pts[i].x, pts[i].y);
	}

	for (i = 1; i <= N; i++)
	{

		next = nextVertex(thePolygon, i, kForwards);
		prev = nextVertex(thePolygon, i, kBackwards);

		angle1 = vertexAngle(thePolygon, prev, i, next);
		angle2 = vertexAngle(thePolygon, next, i, prev);

		myfprintf(out_file, "\n %d %d %d : %g  , %g ", prev, i, next, (double)angle1, (double)angle2);
	}

	angle1 = vertexAngle(thePolygon, 5, 4, 1);
	angle2 = vertexAngle(thePolygon, 1, 4, 5);
	myfprintf(out_file, "\n 5,4,1 ;  : %g  , %g ", (double)angle1, (double)angle2);

	area = PolyArea(thePolygon);
	myfprintf(out_file, "\nThe area is : %g  ", (double)area);

	int convex = convexPoly(thePolygon);
	if (convex)
	{
		myfprintf(out_file, "\nThe polygon is convex.");
	}
	else
	{
		myfprintf(out_file, "\nThe polygon is not convex.");
	}
	fclose(out_file);
	fclose(in_file);
}

/*---------------------------------------------------------------------------
	clear_screen() clears the screen
---------------------------------------------------------------------------*/

void clear_screen(void)

{
	printf("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
}

/*------------------------------------------------------------------------
	This routine gets an integer between hi & lo from the keyboard.
	If the user just hits 'return' then the default value is returned.
------------------------------------------------------------------------*/
int get_choice(int lo, int hi, int dflt)
{

	char buf[255];
	register char *p;
	register int hhi, llo;
	int ret;

	if (lo > hi)
	{
		llo = hi;
		hhi = lo;
	}
	else
	{
		llo = lo;
		hhi = hi;
	}

	*buf = '\0';

	ret = llo - 1;

	while (ret < llo || ret > hhi)
	{

		p = buf;

		fgets(p, 254, stdin);

		/*----------------------------------------
			Skip white space.
		----------------------------------------*/
		while (*p == ' ' || *p == '\t')
			p++;

		/*----------------------------------------
			If RETURN then return default value.
		-----------------------------------------*/
		if (*p == '\n')
		{
			ret = dflt;
			break;
		}

		if (sscanf(p, "%d", &ret) != 1)
			ret = llo - 1;
	}

	return (ret);
}

/*------------------------------------------------------------------------
	This routine gets a double floating point number from the keyboard.
	If the user types a blank line the default value is returned.

	This is used when the user is promted for new parameter values, by
	hitting return he can signal that he wants to keep the original
	value after all.
------------------------------------------------------------------------*/

double get_double(double dflt, int *returnHit)
{

	char buf[255], *p;
	double ret = 0;

	if (returnHit != NULL)
	{
		*returnHit = 0;
	}

	*buf = '\0';
	p = buf;

	while (*buf == '\0')
	{
		fgets(buf, 254, stdin);

		while (*p == ' ' || *p == '\t')
			p++;

		if (*p == '\n')
		{
			if (returnHit != NULL)
			{
				*returnHit = 1;
			}
			ret = dflt;
			break;
		}

		if (sscanf(p, "%lf", &ret) != 1)
			*buf = '\0';
	}

	return (ret);
}

/*---------------------------------------------------------------------------
	This function returns a menu choice from the main menu.
----------------------------------------------------------------------------*/

static char *m1 = "MAIN MENU";
static char *m2 = "Press the number of your choice,";
static char *m3 = "0. Quit Program";
static char *m4 = "1. Change a Parameter";
static char *m5 = "2. Change PID coefficients ";
static char *m6 = "3. Enter a Velocity Function";
static char *m7 = "4. Enter a Setpoint Function";
static char *m8 = "5. Run Simulation";
static char *m9 = "Enter Your Choice, Then RETURN :";

int main_menu(void)

{
	int c = 0;

	clear_screen();

	printf("\t\t\t %s \n\n\t\t %s \n\n\t\t %s \n\t\t %s", m1, m2, m3, m4);
	printf("\n\t\t %s \n\t\t %s \n\t\t %s \n\t\t %s \n\n\t\t %s", m5, m6, m7, m8, m9);

	c = get_choice(0, 5, 6);
	return (c);
}

/*-----------------------------------------------------------------------------
	Get the name of the output file from the user.
-----------------------------------------------------------------------------*/

FILE *get_out_file(char *name, char *title_str)
{

	char fn[255], *fp;
	FILE *ret;

	fp = fn;

	printf("\n\t Enter Name of Output File, OR RETURN for screen output\n");
	printf("\n\tfilename : ");

	fgets(fn, sizeof(fn), stdin);

	while (*fp == '\t' || *fp == ' ')
		fp++;

	if (*fp != '\0' && *fp != '\n')
	{
		if ((ret = fopen(fp, "w")) <= (FILE *)NULL)
		{
			// SysBeep( 5L );
			fprintf(stderr, "Can not open file : %s \n\n", fp);
			ret = get_out_file(name, NULL);
		}
		strcpy(name, fn);
	}
	else
	{
		strcpy(name, "Screen Output");
		ret = stdout;
	}

	if (ret != stdout && title_str != NULL)
	{
		printf("\n Enter a header line for the output file : ");
		fgets(title_str, 254, stdin);
	}

	return (ret);
}

/*-----------------------------------------------------------------------------
	Get the name of the input file from the user.
-----------------------------------------------------------------------------*/

FILE *get_in_file(char *name)
{

	static char fn[255], *fp;
	FILE *ret;

	fp = fn;

	printf("\n\t Enter Name of Input File, OR RETURN for keyboard input\n");
	printf("\n\tfilename : ");

	fgets(fn, sizeof(fn), stdin);

	while (*fp == '\t' || *fp == ' ')
		fp++;

	if (*fp != '\0' && *fp != '\n')
	{
		if ((ret = fopen(fp, "r")) <= (FILE *)NULL)
		{
			// SysBeep( 5L );
			fprintf(stderr, "Can not open file : %s \n\n", fp);
			ret = get_in_file(name);
		}
		strcpy(name, fn);
	}
	else
	{
		ret = stdin;
		strcpy(name, "Keyboard Input");
	}

	return (ret);
}

/*-----------------------------------------------------------------------------
	Print to the file out_file & to stdout if echoing is on.
-----------------------------------------------------------------------------*/
static int _echo = 1;

static void echo(int value)
{

	_echo = value;
}

int myfprintf(FILE *out, char *format, ...)
{

	int ret;
	va_list ap;

	va_start(ap, format);

	if (out != NULL && out != stdout)
	{
		va_start(ap, format);
		ret = vfprintf(out, format, ap);
		va_end(ap);
	}

	if (_echo)
	{
		va_start(ap, format);
		ret = vfprintf(stdout, format, ap);
		va_end(ap);
	}

	return (ret);
}
