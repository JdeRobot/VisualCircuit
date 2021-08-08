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

/**
 * 
 * Tooltip with an arrow
 */
export default function ArrowedTooltip(props: TooltipProps) {
    // Custom CSS classes
    const classes = useStyles();

    return <Tooltip arrow classes={classes} {...props}/>;
}

