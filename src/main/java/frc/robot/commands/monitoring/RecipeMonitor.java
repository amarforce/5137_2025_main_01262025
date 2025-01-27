// src/main/java/frc/robot/commands/monitoring/RecipeMonitor.java
package frc.robot.commands.monitoring;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.logging.RobotLogger;
import java.util.HashMap;
import java.util.Map;

/**
 * Monitors and logs the execution of recipe commands.
 * Provides real-time monitoring of complex command sequences
 * and their component states.
 */
public class RecipeMonitor extends Command {
    private final RobotLogger logger;
    private final Map<String, RecipeStatus> recipeStatuses;
    
    public static class RecipeStatus {
        public String name;
        public String currentStep;
        public boolean isCompleted;
        public long startTime;
        public long duration;
        
        public RecipeStatus(String name) {
            this.name = name;
            this.currentStep = "Initializing";
            this.isCompleted = false;
            this.startTime = System.currentTimeMillis();
        }
    }
    
    public RecipeMonitor() {
        logger = RobotLogger.getInstance();
        recipeStatuses = new HashMap<>();
    }
    
    /**
     * Starts monitoring a new recipe execution
     * @param recipeName Name of the recipe being executed
     */
    public void startRecipe(String recipeName) {
        RecipeStatus status = new RecipeStatus(recipeName);
        recipeStatuses.put(recipeName, status);
        logger.logData("RECIPE_MONITOR", "START", 
            String.format("Recipe: %s started", recipeName));
    }
    
    /**
     * Updates the current step of a recipe
     * @param recipeName Name of the recipe
     * @param step Current step being executed
     */
    public void updateStep(String recipeName, String step) {
        RecipeStatus status = recipeStatuses.get(recipeName);
        if (status != null) {
            status.currentStep = step;
            logger.logData("RECIPE_MONITOR", "STEP", 
                String.format("Recipe: %s, Step: %s", recipeName, step));
        }
    }
    
    /**
     * Marks a recipe as completed
     * @param recipeName Name of the recipe
     * @param success Whether the recipe completed successfully
     */
    public void completeRecipe(String recipeName, boolean success) {
        RecipeStatus status = recipeStatuses.get(recipeName);
        if (status != null) {
            status.isCompleted = true;
            status.duration = System.currentTimeMillis() - status.startTime;
            logger.logData("RECIPE_MONITOR", "COMPLETE", 
                String.format("Recipe: %s, Success: %b, Duration: %dms", 
                    recipeName, success, status.duration));
            recipeStatuses.remove(recipeName);
        }
    }
    
    @Override
    public void execute() {
        // Update SmartDashboard with active recipes
        for (Map.Entry<String, RecipeStatus> entry : recipeStatuses.entrySet()) {
            RecipeStatus status = entry.getValue();
            String prefix = "Recipe/" + status.name + "/";
            
            SmartDashboard.putString(prefix + "Current Step", status.currentStep);
            SmartDashboard.putBoolean(prefix + "Completed", status.isCompleted);
            SmartDashboard.putNumber(prefix + "Duration", 
                (System.currentTimeMillis() - status.startTime) / 1000.0);
        }
    }
    
    @Override
    public boolean isFinished() {
        return false; // Runs continuously
    }
    
    /**
     * Reports an error in recipe execution
     * @param recipeName Name of the recipe
     * @param error Description of the error
     */
    public void reportError(String recipeName, String error) {
        logger.logData("RECIPE_MONITOR", "ERROR", 
            String.format("Recipe: %s, Error: %s", recipeName, error));
        
        // Update dashboard with error
        SmartDashboard.putString("Recipe/Errors/" + recipeName, error);
    }
    
    /**
     * Gets the current status of a recipe
     * @param recipeName Name of the recipe
     * @return Current status or null if recipe not found
     */
    public RecipeStatus getRecipeStatus(String recipeName) {
        return recipeStatuses.get(recipeName);
    }
}s