/****************************************************************************
 *
 * October 2020, updated in January 2023 for integration in the DirectVisualServoing repo
 *
 * Author:
 * Guillaume Caron
 * inspired from CFeatureLuminanceOmni
 *
 *****************************************************************************/

#ifndef CFeatureDefocusedLuminanceFC_h
#define CFeatureDefocusedLuminanceFC_h

#include <visp/vpConfig.h>
#include <visp/vpMatrix.h>
#include <visp3/visual_features/vpFeatureLuminance.h>
#include <visp/vpImage.h>

#include "CCameraThinLensParameters.h"

/*!
  \class CDefocusedLuminance
  \brief Class that defines the defocused luminance, gradient and Laplacian of a point

  \sa CFeatureDefocusedLuminanceFC
*/
class CDefocusedLuminance : public vpLuminance
{
public:
  double x, y;   // point coordinates (in meter)
  double I;      // pixel intensity
  double Ix, Iy; // pixel gradient
  double Z;      // pixel depth
  double Ixx ;   // pixel laplacian
};


/*!
  \class CFeatureDefocusedLuminanceFC
  \brief Class that defines the defocused image brightness visual feature

  for more details see
  G. Caron, RAL/ICRA 2021
*/

class CFeatureDefocusedLuminanceFC : public vpFeatureLuminance
{
protected:
/*  bool recompute_xy;

  int imWidth, imHeight, di, nbri, dj, nbrj, pas;
  int derivativeMaskHalfSize, nbNeigh;
  int nbDim;

  int nbDOF;
  
  */
  //! Store the image (as a vector with intensity and gradient I, Ix, Iy) 
  
  bool DoF[6];
  int nDoF;
  vpColVector pos;
  
private:
  double lambda_;
  

public:
  CDefocusedLuminance *pixInfo ;
	CFeatureDefocusedLuminanceFC() ;
	CFeatureDefocusedLuminanceFC(const CFeatureDefocusedLuminanceFC &f) ;
  vpColVector s;
 
  //! Destructor.
  virtual ~CFeatureDefocusedLuminanceFC() ;
  
  void setDofs(bool tx , bool ty ,bool tz, bool rx , bool ry , bool rz  );
  int getNdofs();
  //void setLambda(double lambda){lambda_=lambda};
  void getDofPos(vpColVector &dof_pos);
  void buildFrom(vpImage<unsigned char> &I);
  vpColVector computeControlLaw(CFeatureDefocusedLuminanceFC &sI, CFeatureDefocusedLuminanceFC &sId);
  //void buildFrom(vpImage<unsigned char> &I, CFeatureDefocusedLuminanceFC *sd) ;

  CFeatureDefocusedLuminanceFC *duplicate() const ;

	
  //void init(int _imHeight, int _imWidth, int _di = 10, int _dj = 10, int _nbri = 10, int _nbrj = 10, int _pas = 1, vpImage<unsigned char> *Imask = NULL, double _rho = 1., int _derivativeMaskSize = 7) ;

  void init(unsigned int _nbr, unsigned int _nbc, double _Z);
	vpMatrix interaction(unsigned int select = FEATURE_ALL);
  void interaction(vpMatrix &L);
  int getDim();
	CFeatureDefocusedLuminanceFC &operator=(const CFeatureDefocusedLuminanceFC &f);



  void setCameraParameters(CCameraThinLensParameters &_cam) ;
  //void resetCameraParameters(CCameraThinLensParameters &_cam) ;

  //void set_DOF(bool un = true, bool deux = true, bool trois = true, bool quatre = true, bool cinq = true, bool six = true);

//private:
  //void cartImagePlaneInteraction(vpMatrix &L);

 public:
  CCameraThinLensParameters cam ;
  


} ;



#endif //CFeatureDefocusedLuminanceFC_h







