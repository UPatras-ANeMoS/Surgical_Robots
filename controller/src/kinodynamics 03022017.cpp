#include "kinodynamics.h"

void constructTransMat(transforms *t_mat,double DH[][4]){
    double c_th,s_th,c_a,s_a;
    for(uint8_t i=0;i<DOF;i++) {
      	c_th=cos(DH[i][3]);
	s_th=sin(DH[i][3]);
	c_a=cos(DH[i][2]);
	s_a=sin(DH[i][2]);
	t_mat->rot_mat[i].set(c_th, -s_th*c_a, s_th*s_a, s_th, c_th*c_a, -c_th*s_a, 0.0, s_a, c_a);
        t_mat->trans_mat[i].set(DH[i][1]*c_th, DH[i][1]*s_th, DH[i][0]);
    }
}

void computeFKine(transforms* transf_mat, transforms* fkine_mat,double DH[][4]){//;cMatrix3d* rot_out, cVector3d* trans_out){
    constructTransMat(transf_mat,DH);
    fkine_mat->rot_mat[0].copyfrom(transf_mat->rot_mat[0]);
    fkine_mat->trans_mat[0].copyfrom(transf_mat->trans_mat[0]);
    cVector3d tmp_vec;
    for(uint8_t i=1;i<DOF;i++){
      //rotation
      fkine_mat->rot_mat[i-1].mulr(transf_mat->rot_mat[i], fkine_mat->rot_mat[i]);
      //translation
      fkine_mat->rot_mat[i-1].mulr(transf_mat->trans_mat[i],tmp_vec);
      fkine_mat->trans_mat[i-1].addr(tmp_vec,fkine_mat->trans_mat[i]);
    }
}

void DH_init(double DH[][4]){
    DH[0][0]=-55.0/2.0; //no change
    DH[0][1]=22.0;  //no change
    DH[0][2]=-C_PI_DIV_2;  //no change
    DH[0][3]=C_PI;  //no change
    DH[1][0]=0.0;
    //*DH[1][0]=-y_motion/2.0;     //prismatic DoF 1
    DH[1][1]=-56.0;   //no change
    DH[1][2]=-C_PI_DIV_2; //no change
    DH[1][3]=C_PI;     //no change
    DH[2][0]=378.5; //DH[2][0]=349.0+(55.0/2.0);
    //*DH[2][0]=144.0+z_motion;     //prismatic DoF 2at fullup extention - 130 is the fixed part
    DH[2][1]=0.0;   //no change
    DH[2][2]=0.0;     //no change
    DH[2][3]=0.0;     //no change
    DH[3][0]=54.0;     //no change
    DH[3][1]=0.0;   //no change
    DH[3][2]=C_PI_DIV_2;     //no change
    DH[3][3]=0.0*C_DEG2RAD;     //rotational DoF 1
    DH[4][0]=0.0;     //no change 178.5
    //DH[4][1]=178.5;   //no change
    DH[4][1]=88.0;
    DH[4][2]=C_PI_DIV_2;     //no change
    DH[4][3]=C_PI_DIV_2+0.0*C_DEG2RAD;     //rotational DoF 2
    DH[5][0]=0.0;
    //*DH[5][0]=-cart_motion/2;     //translational DoF 3 -- assuming 0 at the mid point
    //DH[5][1]=120.0;   //no change
    //DH[5][1]=30.0+8.0+100.0;   //no change for aruco big marker setup
    DH[5][1]=30.0+83.0; //for snake setup
    DH[5][2]=C_PI_DIV_2;     //no change
    DH[5][3]=0.0;     //no change
    DH[6][0]=0.0;     //no change --reach start of robotic snake (shaft+mid distance in one)
    DH[6][1]=double (SHAFT_LENGTH);   //no change
    DH[6][2]=C_PI_DIV_2;     //no change
    DH[6][3]=C_PI_DIV_2;     //no change
}

void DH_change(double DH[][4], double D1,double D2,double th1,double th2,double D3){
    DH[1][0]=D1;
    DH[2][0]=378.5-(Z_motion/2.0)+D2; // -Z_motion/2 < D2 < Z_motion/2
    DH[3][3]=th1;
    DH[4][3]=C_PI_DIV_2+th2;
    DH[5][0]=D3;
}

void print_transform(transforms* transf_mat, uint8_t index){
  if(index<DOF){
    for(uint8_t j=0;j<3;j++){
      cout<< (transf_mat->rot_mat[index].getRow(j)).str(3) << " " << transf_mat->trans_mat[index].get(j) << endl;
    }
    cout<< endl;
  }
  else{
    cout<< "index "<<int (index)<< " is out of DOF bounds"<<endl;
  }
}

void print_DH(double DH[][4]){
  for(uint8_t i=0;i<DOF;i++){
    cout<< double (DH[i][0])<<", " << double (DH[i][1])<<", " << double (DH[i][2])<<", " << double (DH[i][3])<<endl;
  }
  cout<<endl;
}

void robot_RCM(cVector3d pivot_point, cVector3d target_point, double* float_DH){
  //double eucl_dist=sqrt((target_point.z()-pivot_point.z())^2+
  //			(target_point.y()-pivot_point.y())^2+(target_point.x()-pivot_point.x())^2);
  //float_DH[3]=asin((target_point.z()-pivot_point.z())/eucl_dist);
  float_DH[2]=atan2(target_point.y()-pivot_point.y(),target_point.x()-pivot_point.x());
  if(float_DH[2]*C_RAD2DEG>=90.0 || float_DH[2]*C_RAD2DEG<=-90.0)
    float_DH[2]=atan2(target_point.y()-pivot_point.y(),abs(target_point.x()-pivot_point.x()));
  double c1=cos(float_DH[2]);
  double s1=sin(float_DH[2]);
  float_DH[3]=atan2((target_point.z()-pivot_point.z())*s1,target_point.y()-pivot_point.y());
  double c2=cos(float_DH[3]);
  double s2=sin(float_DH[3]);
  float_DH[4]=(target_point.x()+61.8-350*c1*c2+298.5*c1*s2)/c1*c2;
  float_DH[0]=350*s1*c2-298.5*s1*s2+float_DH[4]*c2*s1-target_point.y();
  float_DH[1]=target_point.z()-171.3-298.5*c2-s2*(350+float_DH[4]);
}

void robot_RCM2(double* dh_tmp,cVector3d pivot_point){
  double d3_add=dh_tmp[4];
  double c1=cos(dh_tmp[2]);
  double s1=sin(dh_tmp[2]);
  double c2=cos(dh_tmp[3]);
  double s2=sin(dh_tmp[3]);
  //dh_tmp[4]=(pivot_point.x()-350.0*c1*c2+298.5*c1*s2+61.8)/(c1*c2);
  //dh_tmp[0]=350.0*c2*s1-pivot_point.y()-298.5*s1*s2+dh_tmp[4]*c2*s1;
  //dh_tmp[1]=pivot_point.z()-298.5*c2-350.0*s2-dh_tmp[4]*s2-171.3;
  dh_tmp[4]=(pivot_point.x()-350.0*c1*c2+201.0*c1*s2+78.0)/(c1*c2);
  dh_tmp[0]=350.0*c2*s1-pivot_point.y()-201.0*s1*s2+dh_tmp[4]*c2*s1;
  dh_tmp[1]=pivot_point.z()-201.0*c2-350.0*s2-dh_tmp[4]*s2-207.0;
  dh_tmp[4]+=d3_add;
}


void computePivotLims(double* pivot_DH, double* pivot_angle_lims, double step, cVector3d pivot_point){
  //the expresions here are hard coded although it could be done with symbolic expressions
  double d1_a=0;
  double d2_a=0;
  double d3_a=0;
  double step_rad=step*C_DEG2RAD;
  double c1,c2,s1,s2;
  cout << "Computing new robot angle limits for operation" <<endl;
  double t1=pivot_DH[2];
  double t2=pivot_DH[3];
  c2=cos(t2);
  s2=sin(t2);
  do{
    c1=cos(t1);
    s1=sin(t1);
    d3_a=(pivot_point.x()-350.0*c1*c2+201.0*c1*s2+78.0)/(c1*c2);
    d1_a=350.0*c2*s1-pivot_point.y()-201.0*s1*s2+d3_a*c2*s1;
    d2_a=pivot_point.z()-201.0*c2-350.0*s2-d3_a*s2-207.0;
    //d3_a=(pivot_point.x()-350.0*c1*c2+298.5*c1*s2+61.8)/(c1*c2);
    //d1_a=350.0*c2*s1-pivot_point.y()-298.5*s1*s2+d3_a*c2*s1;
    //d2_a=pivot_point.z()-298.5*c2-350.0*s2-d3_a*s2-171.3;
    t1+=step_rad;
  }while(t1<=pivot_angle_lims[1] && d1_a>=-Y_motion/2.0 && d1_a<=Y_motion/2.0 && d2_a>=0.0 && d2_a<=Z_motion && d3_a<=UPPER_Z_MOTION/2.0);
  t1-=step_rad;
  pivot_angle_lims[1]=t1;
  //
  t1=pivot_DH[2];
  do{
    c1=cos(t1);
    s1=sin(t1);
    d3_a=(pivot_point.x()-350.0*c1*c2+201.0*c1*s2+78.0)/(c1*c2);
    d1_a=350.0*c2*s1-pivot_point.y()-201.0*s1*s2+d3_a*c2*s1;
    d2_a=pivot_point.z()-201.0*c2-350.0*s2-d3_a*s2-207.0;
    t1-=step_rad;
  }while(t1>=pivot_angle_lims[0] && d1_a>=-Y_motion/2.0 && d1_a<=Y_motion/2.0 && d2_a>=0.0 && d2_a<=Z_motion && d3_a<=UPPER_Z_MOTION/2.0);
  t1+=step_rad;
  pivot_angle_lims[0]=t1;
  //
  t1=pivot_DH[2];
  t2=pivot_DH[3];
  c1=cos(t1);
  s1=sin(t1);
  do{
    c2=cos(t2);
    s2=sin(t2);
    d3_a=(pivot_point.x()-350.0*c1*c2+201.0*c1*s2+78.0)/(c1*c2);
    d1_a=350.0*c2*s1-pivot_point.y()-201.0*s1*s2+d3_a*c2*s1;
    d2_a=pivot_point.z()-201.0*c2-350.0*s2-d3_a*s2-207.0;
    t2+=step_rad;
  }while(t2<=pivot_angle_lims[3] && d1_a>=-Y_motion/2.0 && d1_a<=Y_motion/2.0 && d2_a>=0.0 && d2_a<=Z_motion && d3_a<=UPPER_Z_MOTION/2.0);
  t2-=step_rad;
  pivot_angle_lims[3]=t2;
  //
  t2=pivot_DH[3];
  do{
    c2=cos(t2);
    s2=sin(t2);
    d3_a=(pivot_point.x()-350.0*c1*c2+201.0*c1*s2+78.0)/(c1*c2);
    d1_a=350.0*c2*s1-pivot_point.y()-201.0*s1*s2+d3_a*c2*s1;
    d2_a=pivot_point.z()-201.0*c2-350.0*s2-d3_a*s2-207.0;
    t2-=step_rad;
  }while(t2>=pivot_angle_lims[2] && d1_a>=-Y_motion/2.0 && d1_a<=Y_motion/2.0 && d2_a>=0.0 && d2_a<=Z_motion && d3_a<=UPPER_Z_MOTION/2.0);
  t2+=step_rad;
  pivot_angle_lims[2]=t2;
}