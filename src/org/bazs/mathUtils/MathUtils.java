package org.bazs.mathUtils;

public class MathUtils
{
   public static boolean equals(double a, double b, double eps)
   {
      return (Math.abs(a - b) < eps);
   }
}
