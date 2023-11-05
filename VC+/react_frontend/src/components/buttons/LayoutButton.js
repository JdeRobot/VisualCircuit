import * as React from 'react';
import '../../styles/styles.css';
import PropTypes from "prop-types";
import FactCheckIcon from "@mui/icons-material/FactCheck";
import StyleIcon from "@mui/icons-material/Style";
import AssistantIcon from "@mui/icons-material/Assistant";
import ViewColumnIcon from '@mui/icons-material/ViewColumn';

const LayoutButton = (props) => {
  const { changeVisualization, visualization, toggleDisplayVisual , toggleDisplayEditor, toggleDisplayDirectory, toggleChangeBroads, broad} = React.useContext(
    props.context
  );

  const OpenBoth = () => {
    toggleDisplayVisual('flex');
    toggleDisplayEditor('flex');
    toggleDisplayDirectory('flex');
    toggleChangeBroads({
        ...broad,
        broad1: "20%",
        broad2: "50%",
    });
    changeVisualization({
          ...visualization,
          gazebo: true,
          specific: true,
          console: true,
    });
  };

  // Call OpenBoth to apply its functionality immediately upon rendering
  React.useEffect(() => {
    OpenBoth();
  }, []);

  return null; // Since you don't want any buttons or UI, return null
};

export default LayoutButton;
