#include "gradientDescent.hpp"

VanillaGradientDescent::VanillaGradientDescent(){}
void VanillaGradientDescent::setVar(double m, double Dx, double Alpha){
  M = m;
  alpha = Alpha;
  dx = Dx;
  Mb=m;
  T=0;
}

void VanillaGradientDescent::updateMb(){
  	Mb=M+dx;
  }
  double VanillaGradientDescent::getMb(){
  	return Mb;
  }
 double VanillaGradientDescent::gradient(double Tt){
  	double g = (Tt-T)/dx;
  	std::cout<<"-----------\n"<<"g: "<<g<<std::endl;
  	std::cout<<"Tt: "<<Tt<<"T: "<<T<<std::endl;
  	return g;
  }
  double VanillaGradientDescent::updateM(double Tt){
        if(T==0){
            T=Tt;
            return M;
        }
  	M = M-alpha * this->gradient(Tt);
  	T=Tt;
  	return M;
  }
