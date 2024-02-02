import { Theme } from "@material-ui/core/styles/createMuiTheme";
import makeStyles from "@material-ui/core/styles/makeStyles";
import Tooltip, { TooltipProps } from "@material-ui/core/Tooltip/Tooltip";


const useStyles = makeStyles((theme: Theme) => ({
    arrow: {
        color: theme.palette.common.white,
    },
    tooltip: {
        backgroundColor: theme.palette.common.white,
        color: theme.palette.common.black
    }
}));

// Function for unit conversion in a given input string
export const unitConversion  = (input: string): string => {
  
  // Define a multiplier object for unit conversions
  const multiplier: { [key: string]: number } = {
        'k': 1000,
        'M': 1000000,
        'm': 0.001,
        'K': 1000,
      };
    
      const values = input.split(',');  // Split input string by commas to handle multiple values

      // Map over each value to perform unit conversion
      const convertedValues = values.map((value) => {

        /*
          Regular expression to match and capture components of a value:
          - ([\d,]*\.?\d+): Captures the numeric part of the value, which may include digits and commas
                            allowing for thousands separators, and an optional decimal point and fractional part.
          - ([kKmM]?): Captures the unit part of the value, which may be 'k', 'K', 'm', or 'M', and is optional.
          - $: Asserts the end of the string, ensuring the match occurs at the end of the value.
          The result is stored in the 'match' variable, containing an array with the full matched value,
        */
        const match = value.match(/([\d,]*\.?\d+)([kKmM]?)$/);
        
        // Check if a match is found
        if (match) {

          // Extract numeric value and unit from the match
          const numericValue = parseFloat(match[1]);
          const unit = match[2] || '';

          // Check if the unit is present in the multiplier object
          if (multiplier.hasOwnProperty(unit)) {

            // Perform unit conversion and return the result as a string
            const result = numericValue * multiplier[unit]; 
            return result.toString();

          } else {

            // If unit not found, return the numeric value as a string
            return numericValue.toString();

          }
        }
    
        /*
          Regular expression to add commas to the numeric part of the given 'value'.
          - (\d): Captures a single digit.
          - (?=(\d{3})+(?!\d)): This positive lookahead assertion ensures that the digit being considered is part of a sequence where every three digits are followed by another group.
          - g: Global flag, ensuring this replacement is applied throughout the 'value'.
          -'$1,': Replaces each captured digit with itself followed by a comma, achieving the separator effect.
        */
        return value.replace(/(\d)(?=(\d{3})+(?!\d))/g, '$1,');
      });

      return convertedValues.join(','); // Join the converted values back into a comma-separated string and return
  };

/**
 * 
 * Tooltip with an arrow
 */
export default function ArrowedTooltip(props: TooltipProps) {
    // Custom CSS classes
    const classes = useStyles();

    return <Tooltip arrow classes={classes} {...props}/>;
}

