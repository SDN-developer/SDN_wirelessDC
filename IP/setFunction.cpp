
// Headers
#include "../basic_lib.h"
#include "IP.h"

// IP class
void IP::setIP(string str){

	// Check and convert
	int i, cnt=0, tmp=0;
	for(i=0;i<str.size();i++){
		if(str[i]=='.'){
			if(tmp>255||cnt==3) break;
			byte[cnt]=tmp;
			tmp=0;
			cnt++;
		}
		else if(str[i]>='0'&&str[i]<='9') tmp=tmp*10+str[i]-'0';
		else break;
	}
	if(i<str.size()) printf("Error: illegal IP format\n");
	else{
		byte[cnt]=tmp;
		fullIP=str;
	}
}
void IP::setIP(int b0, int b1, int b2, int b3){

	int i;
	char stmp[4];
	if(b0>=0&&b0<=255 && b1>=0&&b1<=255
	&& b2>=0&&b2<=255 && b3>=0&&b3<=255){
		byte[0]=b0;
		byte[1]=b1;
		byte[2]=b2;
		byte[3]=b3;
		fullIP="";
		for(i=0;i<4;i++){
			if(i) fullIP+=".";
			sprintf(stmp, "%d", byte[i]);
			fullIP+=stmp;
		}
	}
	else printf("Error: illegal IP format\n");
}
