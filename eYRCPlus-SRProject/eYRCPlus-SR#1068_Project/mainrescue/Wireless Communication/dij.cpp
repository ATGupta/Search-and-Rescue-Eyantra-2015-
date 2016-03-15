 /*
 Team Id :          eYRCPlus-SR#1068 
 Author List :      Bittu Chauhan
 Filename:          dij.cpp
 Theme:             Search and Rescue
 Functions:         rem,rem1,add,shortest_path
 Global Variables:	dist1,dist,path,original,n,tot
 */ 
#define MAX_NODES 200
#define INFINITE 9999
unsigned int n=MAX_NODES;
int dist1[4][2];                // node config of temporary removed node
int dist[165][2]={{1,2},{1,11},{2,1},{2,3},{3,2},{3,4},{3,13},{4,3},{4,5},{5,4},{5,6},{5,15},{6,5},{6,7},{7,6},{7,8},
    {7,17},{8,7},{8,9},{9,8},{9,19},{11,1},{11,21},{13,3},{13,23},{15,5},{15,25},{17,7},{17,27},{19,9},{19,29},
	{21,11},{21,22},{21,31},{22,21},{22,23},{23,22},{23,24},{23,13},{23,33},{24,23},{24,25},{25,24},{25,26},{25,15},{25,35},
	{26,25},{26,27},{27,26},{27,28},{27,17},{27,37},{28,27},{28,29},{29,28},{29,19},{29,39},
	{31,21},{31,40},{33,42},{33,23},{35,25},{35,44},{37,27},{37,46},{39,29},{39,48},
    {40,31},{40,41},{40,51},{41,40},{41,42},{42,41},{42,43},{42,33},{42,53},{43,42},{43,44},{44,43},{44,45},{44,35},{44,55},
	{45,44},{45,46},{46,45},{46,47},{46,37},{46,57},{47,46},{47,48},{48,47},{48,39},{48,59},{48,49},{49,48},
	{51,40},{51,61},{53,42},{53,63},{55,65},{55,44},{57,67},{57,46},{59,69},{59,48},
	{61,51},{61,62},{61,71},{62,61},{62,63},{63,62},{63,64},{63,53},{63,73},{64,63},{64,65},{65,64},{65,66},{65,55},{65,75},
	{66,65},{66,67},{67,66},{67,68},{67,57},{67,77},{68,67},{68,69},{69,68},{69,59},{69,79},
	{71,61},{71,81},{73,83},{73,63},{75,65},{75,85},{77,67},{77,87},{79,69},{79,89},
	{81,71},{81,82},{82,81},{82,83},{83,82},{83,84},{83,73},{84,83},{84,85},{85,84},{85,86},{85,75},{85,90},
	{86,85},{86,87},{87,86},{87,88},{87,77},{88,87},{88,89},{89,88},{89,79},{90,85}
	};                         // arena configration
  int path[MAX_NODES],original[MAX_NODES];
//int arr[4]={1,2,3,2};
unsigned int tot=0;
typedef enum{permanent,tentative} node_label;
struct state
   {
     unsigned int predecessor;
     unsigned int length;
      node_label label;
   }state[MAX_NODES];
/*
Function Name: rem
Input :       b(node number)
Output :      none
Logic:       it searches for the node in arena and sets it value to 0 to make sure that the node can never be used again in path
Example Call:rem(55)
*/
void rem(int b)
{
	int j=0;
    for(int i=0;i<175;i++)
    {
        if(dist[i][0]==b||dist[i][1]==b)
        {
            //cout<<"\nREMOVED="<<i<<" "<<dist[i][0]<< " "<<dist[i][1];
			dist[i][0]=0;
            dist[i][1]=0;
        }


    }
}
/*
Function Name: rem1
Input :       b(node number)
Output :      none
Logic:       it searches for the node in arena and copies it config and sets it value to 0 to make sure that the node can never be used again in path temporarily
Example Call:rem1(55)
*/
void rem1(int b)
{
	int j=0;
	//lcd_string("removed");
	//lcd_print(2,1,b,3);
	for(int i=0;i<175;i++)
	{
		if(dist[i][0]==b||dist[i][1]==b)
		{
			//cout<<"\nREMOVED="<<i<<" "<<dist[i][0]<< " "<<dist[i][1];
			dist1[j][0]=dist[i][0];
			dist1[j][1]=dist[i][1];
			j++;
			dist[i][0]=180;
			dist[i][1]=180;
		}
	}
	while(j<4)
	{
		dist1[j][0]=0;
		dist1[j][1]=0;
		j++;
		
	}
}
/*
Function Name: add
Input :       k(node number)
Output :      none
Logic:       it searches for the node in config table and copies the config in adj matrix to make sure that the node can now be used again in path
Example Call:add(55)
*/
int add(int k)
{
	int j=0;
	for(int i=0;i<175;i++)
	{
		if(dist[i][0]==180&&dist[i][1]==180)
		{
			//cout<<"\nREMOVED="<<i<<" "<<dist[i][0]<< " "<<dist[i][1];
			dist[i][0]=dist1[j][0];
			dist[i][1]=dist1[j][1];
			j++;
		}
	}
}
/*
Function Name: shortest_path
Input :       s(source),d(destination)
Output :      none
Logic:       standard implementation of dijkstra algorithm
Example Call:shortest_path(55,68)
*/
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
			for(part=0;part<163;part++)
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
		original[j]=path[j];
	}
	for(unsigned int j=0;j<tot-1;j++)
	{
		if((path[j]/10==path[j+1]/10)&&(path[j]<path[j+1]))
		path[j]='S';
		else if((path[j]/10==path[j+1]/10)&&(path[j]>path[j+1]))
		path[j]='N';
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
			path[j]=3;
		}
		else if(path[j]==270||path[j]==-90)
		{
			path[j]=1;
		}
		else if(path[j]==0)
		{
			path[j]=4;
		}
		else
		{
			path[j]=2;
		}
	}

}

/*
int main()
{
    int x=0;
    shortest_path(49,39);
    for(int i=0;i<tot-1;i++)
    {
        x+=path[i];
        cout<<path[i]<<" ";
    }
    cout<<"turn="<<4-(x%4)<<endl;

    rem(39);

    shortest_path(49,1);
    x=0;
    for(int i=0;i<tot-1;i++)
    {
        x+=path[i];
        cout<<path[i]<<" ";
    }
    cout<<"turn="<<4-(x%4)<<endl;
    cout<<endl;
}*/
