import * as React from "react";
import Toolbar from "@mui/material/Toolbar";
import { Box, TextField } from "@mui/material";
import RoboticsTheme from "../../RoboticsTheme";
import PropTypes from "prop-types";
import { SaveButton } from "../../buttons/SaveButton";
import FileManagementButton from "../../buttons/FileManagementButton";
import RAMPlay from "../../buttons/RAM/RAMPlay";
import RAMPause from "../../buttons/RAM/RAMPause";
import RAMStop from "../../buttons/RAM/RAMStop";
import SaveFeedbackIcon from "../../visualizers/RAM/RAMSaveFeedbackIcon";
import LayoutButton from "../../buttons/LayoutButton";
import ViewsLayoutButton from "../../buttons/ViewsLayoutButton";
import WorldSelector from "../../buttons/RAM/RAMWorldSelector";
import ChooseWorld from "../../buttons/RAM/RAMChooseWorld";
import ApplicationSelector from "../../buttons/RAM/RAMApplicationSelector";

function RAMExerciseControl(props) {
  const { filename, setFileName } = React.useContext(props.context);

  return (
    <RoboticsTheme>
      <Toolbar
        sx={{
          display: "flex",
          flexWrap: "nowrap",
          justifyContent: "space-evenly",
          alignItems: "center",
          m: 1,
          p: 2,
          border: "2px solid #d3d3d3",
        }}
      >
        <Box id={"editor-control"} sx={{alignItems: "center", justifyContent: "center", flexWrap: "wrap"}}>
          <Box sx={{display:"flex", alignItems: "center", justifyContent: "center", flexWrap: "wrap"}}>
              <ChooseWorld context={props.context}></ChooseWorld>
              <FileManagementButton context={props.context}/>
              <SaveFeedbackIcon context={props.context} />
          </Box>
        </Box>
        <Box sx={{display: "flex", alignItems: "center", justifyContent: "center", flexWrap: "wrap"}}>
            <WorldSelector context={props.context}></WorldSelector>
            <ApplicationSelector context={props.context}></ApplicationSelector>
        </Box>
        <Box id={"robot-control"} sx={{flexWrap: "wrap"}}>
          <RAMPlay context={props.context}></RAMPlay>
          <RAMPause></RAMPause>
          <RAMStop></RAMStop>
        </Box>
        <Box id={"Sim-console-control"} sx={{ zIndex: 2, display: "flex", alignItems: "center" }}>
          <ViewsLayoutButton context = {props.context}/>
          <LayoutButton context={props.context}/>
        </Box>
      </Toolbar>
    </RoboticsTheme>
  );
}
RAMExerciseControl.propTypes = {
  context: PropTypes.any.isRequired,
  specificConfiguration: PropTypes.any,
};

export default RAMExerciseControl;
