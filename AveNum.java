import java.util.Scanner;

import edu.wpi.first.units.PerUnit;

class AveNum{
    static Scanner scanner = new Scanner(System.in);
    public static double ave(int sum,int count){
        if(count == 0){
            return 0;
        }
        return (double)sum/count;
    }
    public static double num4Ave(int num1, int num2, int num3, int num4){
        return ave(num1 + num2 + num3 + num4, 4);
    }
    public static void aveGrades(){
        int sum=0;
        for(int i=0;i<4;i++){
            System.out.print("Enter grade " + (i+1) + ": ");
            int grade = scanner.nextInt();
            sum += grade;
        }
        System.out.println("Average grade: " + num4Ave(sum,0,0,0));
    }
}