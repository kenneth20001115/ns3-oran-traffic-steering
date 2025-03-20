#include <algorithm>
#include <iostream>
#include <vector>

#ifndef GRADIENTDESCENT_HPP
#define GRADIENTDESCENT_HPP

class VanillaGradientDescent {
public:
  VanillaGradientDescent();
  
  void setVar(double m, double Dx, double Alpha);
  
  void updateMb();
  
  double getMb();
  double gradient(double Tt);
  double updateM(double gradient);
  void run();
  
protected:
  double M,Mb, dx,alpha,T;
  std::vector<long double> solution;

  long double checkNewValue(long double newValue);
  void showSolution();

  long double uniformRandomNum(long double MinValue, long double MaxValue);
  std::vector<long double> uniformVectorRandomNum(int size);
};

#endif
