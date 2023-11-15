package frc.team4276.frc2024.subsystems;

public class Example {

    public void isGreater(double val){

        double min = 5;

        if (val > min){
            // the variable val is greater than min
        }

    }

    public boolean isEqualTo(double val){
        double des = 7;

        if (val == des){
            // the value passed into the function is equal to the desired variable
            // return value of function is true and skip the rest of the function code
            return true;
        }

        // the value passed into the function is NOT equal to the desired variable
        // function returns false because the passed value is not equal to the desired value
        return false;
    }

    public boolean inRange(double min, double max, double val){
        if ((val > min) && (val < max)){
            return true;
        }
        
        return false;
    }

    public void examples(boolean a, boolean b){
        if (a || b){
            // run this if a or b is true
        }

        double x = 4.276;

        boolean isInRange = inRange(2.54, 16.78, x);

        // isInRange is true!!

        System.out.println(isInRange);
    }

    public void example(boolean a, boolean b){
        if (a || b){
            // run this if a or b is true
        }

        double x = 42.76;

        boolean isInRange = inRange(2.54, 16.78, x);

        // isInRange is false D:

        System.out.println(isInRange);
    }
    
    
}
