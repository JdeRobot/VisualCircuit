import * as React from 'react';
import '../../styles/styles.css';
import PropTypes from "prop-types";
import { Button } from "@mui/material";
import FactCheckIcon from "@mui/icons-material/FactCheck";
import StyleIcon from "@mui/icons-material/Style";
import AssistantIcon from "@mui/icons-material/Assistant";
import ViewColumnIcon from '@mui/icons-material/ViewColumn';

const LayoutButton = (props) => {
  const [open, setOpen] = React.useState(false);
  const { changeVisualization, visualization, toggleDisplayVisual , toggleDisplayEditor, toggleDisplayDirectory, toggleChangeBroads, broad} = React.useContext(
    props.context
  );

  const handleOpen = () => {
    setOpen(!open);
  };

  const OpenEditor = () => {
    toggleDisplayVisual('none');
    toggleDisplayEditor('flex');
    toggleDisplayDirectory('flex');
    toggleChangeBroads({
        ...broad,
        broad1: "20%",
        broad2: "100%",
    });
    changeVisualization({
          ...visualization,
          gazebo: false,
          specific: false,
          console: false,
    });
    setOpen(false);
  };

  const OpenVisual = () => {
    toggleDisplayVisual('flex');
    toggleDisplayEditor('none');
    toggleDisplayDirectory('none');
    toggleChangeBroads({
        ...broad,
        broad1: "0%",
        broad2: "0%",
    });
    changeVisualization({
          ...visualization,
          gazebo: true,
          specific: true,
          console: true,
        });
    setOpen(false);
  };

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
    setOpen(false);
  };

  return (
    <div>
      <Button
      id={"Layout_button"}
      size={"medium"}
      variant="contained"
      color={"secondary"}
      component="span"
      sx={{ m: 1 }}
      onClick={handleOpen}
      startIcon={<FactCheckIcon/>}
      >Layouts</Button>
      {open ? (
        <ul className="menu">
          <li className="menu-item">
            <Button
            id={"Only_editor_button"}
            size={"medium"}
            variant="contained"
            color={"secondary"}
            component="span"
            sx={{ m: 1 }}
            onClick={OpenEditor}
            startIcon={<StyleIcon/>}
            >Only editor</Button>
          </li>
          <li className="menu-item">
            <Button
             id={"Only_visual_button"}
             size={"medium"}
             variant="contained"
             color={"secondary"}
             component="span"
             sx={{ m: 1 }}
             onClick={OpenVisual}
             startIcon={<AssistantIcon/>}
             >Only visual</Button>
          </li>
          <li className="menu-item">
            <Button
             id={"Both_button"}
             size={"medium"}
             variant="contained"
             color={"secondary"}
             component="span"
             sx={{ m: 1 }}
             onClick={OpenBoth}
             startIcon={<ViewColumnIcon/>}
             >Visual and Editor</Button>
          </li>
        </ul>
      ) : null}
    </div>
  );
};

export default LayoutButton;