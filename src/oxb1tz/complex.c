#include "oxb1tz/complex.h"

const oxb_complex OXB_COMPLEX_IDENTITY = OXB_VEC2_INIT(1.0f, 0.0f);

/*
 src : https://en.wikipedia.org/wiki/Euler%27s_formula
 
 the idea is based on the Euler's formula 

  e ^ i*θ = cos(θ) + i sin(θ)
 
  e = euler's number (≈2.71828)
  i = imaginary (√-1 or (complex){0, 1})
 
 
 when you use logarithm, you can express it as
 inverse trigonometry

  ln(e ^ i*radians) where you can extract the radians as imaginary part


 Multiplication :
 
  multiplication of a unit complex numbers means adding the
  radians angle.
  
  similar to exponent identity :
  (a ^ b) * (a ^ c) = a ^ (b + c)
  
  for complex :
  (e ^ iA) * (e ^ iB) = e ^ (iA + iB)
  the angle of both complex numbers (iA and iB imaginary angles) accumulate.


 example C++:

  double theta = M_PI_2;
  std::complex<double> i(0, 1);
  std::complex<double> unit_vector = std::exp(i * theta);
  std::complex<double> inverse = std::log(unit_vector);

  std::cout << unit_vector << '\n';
  std::cout << inverse << '\n';

*/


/*
 Minimal Version of Euler's Formula works
 by avoiding the expensive exponential and logarithm
 computation and exploit the fact that e ^ 0 == 0 
 and ln(1) == 0. both of them can be implicit and cancels
 out each other.
*/


/* 
 complex rotate uses a simplifcation that eliminates the 
 exponential.

 complex exponential :
  input (a, b)
  output : (e ^ a * cos(b), e ^ a * sin(b))
 
 idea :
  e ^ i*θ = cos(θ) + i sin(θ)
 
 simplification : 
  since (e ^ 0 = 1.0) you can completely eliminate the exponential
              leaving only the trigonometry part of imaginary
*/
oxb_complex oxb_complex_rotate(const float a) {
 float s, c;
 oxb_sincos(a, &s, &c);
 return (oxb_complex)OXB_VEC2_INIT(c, s);
}
/*
 complex angle uses simplification (same tricks, to eliminate the logarithm)
 complex logaritm:
 
 
  input (input_complex)(a, b)
  
            ln(sqrt(dot(input_complex, input_complex)))
  output : (_____________________________________,  atan2(b, a))
                             2.0

  output : (ln(sqrt(a * a + b * b)) * 0.5,  atan2(b, a))
  
  idea :
  
   ln(e ^ i*θ) = iθ
 
 simplification : 
  earlier, in this formula (e ^ i*θ), we input the angle_theta as imaginary (i*θ)
  so we expect to return it as imaginary. because of that, we can ignore the
  logarithm part completely.
*/
float oxb_complex_angle(const oxb_complex a) {
 return oxb_atan2(oxb_complex_imag(a), oxb_complex_real(a));
}

