import * as React from "react";
import { Box } from "@mui/material";
import RAMExerciseControl from "../../common/RAM/RAMExerciseControl";
import AceEditorRobot from "../../exercises/AceEditorRobot";
import FlexContainer from "../../exercises/FlexContainer";

import PropTypes from "prop-types";

import { LinterModal } from "../../modals/LInterModal";
import { Visualization } from "../Visualization";
import { Frequencies } from "../../visualizers/RAM/RAMFrequency";
import RAMImgCanvas from "../../visualizers/RAM/RAMImgCanvas";

import DirectoryView from "../../exercises/DirectoryView";

function FollowLineExerciseView(props) {
  const { broad } = React.useContext(props.context);
  return (
    <Box id="exercise-view">
      <RAMExerciseControl
        context={props.context}
      />
      <Box
        sx={{
          display: "flex",
          justifyContent: "space-around",
          p: 1,
          m: 1,
          background: "linear-gradient(#EOECDE, #FFFFFF)",
        }}
      >
      <FlexContainer row={true} broad={broad.broad1}>
        <DirectoryView context={props.context}/>
        <FlexContainer row={true} broad={broad.broad2}>
            <AceEditorRobot context={props.context}/>
            <Visualization context={props.context}/>
        </FlexContainer>
      </FlexContainer>
      </Box>
      {/*<Frequencies></Frequencies>*/}
      {/*<LinterModal context={props.context}></LinterModal>*/}
    </Box>
  );
}
FollowLineExerciseView.propTypes = {
  context: PropTypes.any,
};

export default FollowLineExerciseView;
