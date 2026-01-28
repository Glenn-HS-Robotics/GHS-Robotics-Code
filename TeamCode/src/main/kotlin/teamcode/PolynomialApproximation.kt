package teamcode

import java.util.ArrayList
import kotlin.math.pow

class PolynomialApproximation(vararg coefficient: Double) {
    val coefficients = coefficient

    fun approximate(input: Double): Double{
        var total = 0.0
        coefficients.forEachIndexed { index, coefficient ->
            if(index == 0)
                total += coefficient
            else
                total += coefficient * input.pow(index)
        }

        return total

    }

}