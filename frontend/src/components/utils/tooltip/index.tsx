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

export const unitConversion  = (input: string): string => {
    const multiplier: { [key: string]: number } = {
        'k': 1000,
        'M': 1000000,
        'm': 0.001,
        'K': 1000,
      };
    
      const values = input.split(',');

      const convertedValues = values.map((value) => {
        const match = value.match(/([\d,]*\.?\d+)([kKmM]?)$/);
        
        if (match) {
          const numericValue = parseFloat(match[1]);
          const unit = match[2] || '';
          if (multiplier.hasOwnProperty(unit)) {
            const result = numericValue * multiplier[unit];
            return result.toString();
          } else {
            return numericValue.toString();
          }
        }
    
        return value.replace(/(\d)(?=(\d{3})+(?!\d))/g, '$1,');
      });
      return convertedValues.join(',');
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

