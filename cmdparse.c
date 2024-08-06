#include <ctype.h>
#include <stddef.h> 

#include "cmdparse.h"

int getCommandNum( CMD_ENTRY *cmds, char *inputStr, int *length ) {

	int i=0, len=0, ret=0 ;	
	
	for( i=0; cmds[i].name!=NULL && len==0; i++ ) {
		len = match( cmds[i].name, inputStr ) ;
	}
	
	*length = len ;
	
	if(len>0) ret=cmds[i].id ;
	
	return( ret ) ;
}


	
int match( char *token, char *string ) {

	int	i=0, j=0, ret=0;
	
	while( string[i] && isspace((int) string[i]) )
		i++ ;
		
	while( token[j] && string[i] && token[j]==string[i] ) {
		i++ ;
		j++ ;
	}
	
	if( token[j]=='\0' && i>0 && j>0 ) ret=i ;
	
	return( ret ) ;
	
}