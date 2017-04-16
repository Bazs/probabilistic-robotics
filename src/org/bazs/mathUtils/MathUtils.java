package org.bazs.mathUtils;

public class MathUtils
{
   private MathUtils()
   {
      // Private constructor, utility class
   }

   public static boolean doubleEquals(double a, double b, double eps)
   {
      return (Math.abs(a - b) < eps);
   }

   public static double calcEntropyOfDiscreteProbDist(Double[] probDist)
   {
      double entropy = 0.0;

      for (double x : probDist)
      {
         entropy -= x * Math.log(x) / Math.log(2);
      }

      return entropy;
   }
}
