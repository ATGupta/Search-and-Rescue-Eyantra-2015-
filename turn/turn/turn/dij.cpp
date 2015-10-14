#define MAX_NODES 110
#define INFINITE 9999
unsigned int n=MAX_NODES;
unsigned int dist[175][2]={{1,2},{1,11},{2,1},{2,13},{2,3},{3,2},{3,4},{3,16},{4,3},{4,18},{10,11},{11,1},{11,10},{11,21},{12,13},{12,32},{13,12},{13,2},{13,14},{14,22},
	{15,16},{15,24},{16,15},{16,17},{17,16},{17,37},{18,4},{18,19},{18,25},{19,18},{20,21},{21,20},{21,11},{21,31},{22,14},{22,34},{22,23},{23,22},{23,41},
	{23,24},{24,15},{24,23},{24,35},{25,18},{25,26},{25,38},{26,25},{30,31},{31,21},{31,30},{31,51},{32,12},{32,33},{33,32},{33,34},{33,52},{34,33},{34,22},
	{35,24},{35,36},{36,35},{36,37},{36,55},{37,36},{37,17},{38,25},{38,56},{38,39},{39,38},{40,41},{40,53},{41,23},{41,40},{41,42},{42,41},{42,54},
	{50,51},{51,31},{51,50},{51,52},{51,71},{52,51},{52,33},{52,53},{52,73},{53,40},{53,52},{53,60},{54,42},{54,55},{54,62},{55,54},{55,36},{55,56},{55,76},
	{56,55},{56,38},{56,71},{60,53},{60,61},{61,60},{61,62},{61,83},{62,61},{62,54},{70,71},{71,70},{71,51},{71,81},{72,73},{72,92},{73,52},{73,72},
	{73,74},{74,73},{74,82},{75,84},{75,76},{76,55},{76,75},{76,77},{77,76},{77,97},{78,56},{78,85},{78,79},{79,78},{80,81},{81,80},{81,71},{81,91},
	{82,74},{82,94},{82,83},{83,61},{83,82},{83,84},{84,75},{84,95},{85,78},{85,98},{85,86},{86,85},{90,91},{91,81},{91,90},{91,100},{92,72},{92,93},
	{93,92},{93,101},{93,94},{94,82},{94,93},{95,84},{95,96},{96,95},{96,97},{96,102},{97,77},{97,96},{98,85},{98,99},{98,103},{99,98},{100,91},
	{100,101},{101,93},{101,100},{101,102},{102,96},{102,101},{102,103},{103,98},{103,102}};
 int path[MAX_NODES];
//int arr[4]={1,2,3,2};
unsigned int tot=0;
typedef enum{permanent,tentative} node_label;
struct state
   {
     unsigned int predecessor;
     unsigned int length;
      node_label label;
   }state[MAX_NODES];

void shortest_path(unsigned int s,unsigned int t)
{
	
   unsigned int f=0,part;
   unsigned int i,k,min;
   struct state *p;
   for(p=&state[0];p<&state[n];p++)
   {
		p->predecessor=-1;
		p->length=INFINITE;
		p->label=tentative;
   }
   state[t].length=0;
   state[t].label=permanent;
   k=t;
  
   do
   {
	   //lcd_string("do started");
		for(i=0;i<n;i++)
		{
			f=0;
			for(part=0;part<174;part++)
			{
			
				if((dist[part][0]==k)&&(dist[part][1]==i))
				{
					f=1;
					break;
					
				}
			}
			//lcd_string("break");
			
			 if(f==1 && state[i].label==tentative)
			{
				if(state[k].length+1<state[i].length)
				{
					state[i].predecessor=k;
					state[i].length=state[k].length+1;
					
				}
		    }
		}
		//lcd_string("to");
		k=0;min=INFINITE;
		for(i=0;i<n;i++)
		{
			if(state[i].label==tentative && state[i].length<min)
			{
				min=state[i].length;
				k=i;	
			}
			
		}
		state[k].label=permanent;
   }while(k!=s);
   
   i=0;k=s;
   do
   {
		path[i++]=k;
		
		k=state[k].predecessor;
   }while(k!=t);
   path[i++]=t;
   tot=i;
	
	for(unsigned int j=0;j<tot-1;j++)
	{
		if((path[j]/10==path[j+1]/10)&&(path[j]<path[j+1]))
		path[j]='N';
		else if((path[j]/10==path[j+1]/10)&&(path[j]>path[j+1]))
		path[j]='S';
		else if(path[j]>path[j+1])
		path[j]='W';
		else if(path[j]<path[j+1])
		path[j]='E';

	}
	for(unsigned int j=0;j<tot-1;j++)
	{
		if((char)path[j]=='N')
		path[j]=0;
		if((char)path[j]=='E')
		path[j]=1;
		if((char)path[j]=='S')
		path[j]=2;
		if((char)path[j]=='W')
		path[j]=3;
	}
	int current =0;
	int temp=0;
	for(unsigned int j=0;j<tot-1;j++)
	{
		temp=current;
		current=path[j];
		path[j]=(path[j]-temp)*90;
		//cout<<path[j]<<" ";
	}
	for(unsigned int j=0;j<tot-1;j++)
	{
		if(path[j]==90||path[j]==-270)
		{
			path[j]=1;
		}
		else if(path[j]==270||path[j]==-90)
		{
			path[j]=3;
		}
		else if(path[j]==0)
		{
			path[j]=2;
		}
		else
		{
			path[j]=4;
		}
	}
	 
}


