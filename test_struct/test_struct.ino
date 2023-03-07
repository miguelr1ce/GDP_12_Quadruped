
#define itr 4 //Number of iteration

struct crawl{
  double FL[4][itr]={{0,1,2,3},{0,1,2,3},{0,1,2,3},{0,1,2,3}}; //row: X,Y,Z,Rotation; Column: Iterations
  double FR[4][itr]; 
  double RL[4][itr]; 
  double RR[4][itr]; 
};

struct trot{
  double FL[4][itr]={{4,5,6,7},{4,5,6,7},{4,5,6,7},{4,5,6,7}}; //row: X,Y,Z,Rotation; Column: Iterations
  double FR[4][itr]; 
  double RL[4][itr]; 
  double RR[4][itr]; 
};

crawl data;
trot datb;


//data A;
leg itt[3];

void setup() {
  Serial.begin(115200);
  for(int i=0;i<4;i++){  
  for (int j=0;j<itr;j++){
    Serial.print(data.FL[i][j]+datb.FL[i][j]);
    Serial.print('\t');
  }
  Serial.print('\n');
}

}

void loop() {
  // put your main code here, to run repeatedly:
}
