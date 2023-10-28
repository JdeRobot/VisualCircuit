import * as React from "react";
import PropTypes from "prop-types";
import CommsManager from "../libs/comms_manager";
import { useState } from "react";
import { saveCode } from "../helpers/utils";
const NewManagerExerciseContext = React.createContext();

export function ExerciseProvider({ children }) {
  const ramHost = window.location.hostname;
  const ramPort = 7163;
  CommsManager(`ws://${ramHost}:${ramPort}`);

  const csrftoken = document.querySelector('[name=csrfmiddlewaretoken]').value;

  const [displayEditor, setDisplayEditor] = useState('flex');

  const [displayVisual, setDisplayVisual] = useState(100);

  const [displayDirectory, setDisplayDirectory] = useState(35);

  const [broad, setBroads] = useState({
    broad1: "20%",
    broad2: "50%",
  });
  const [projectURL, setProjectURL] = useState("");

  const [selectedNodeId, setSelectedNodeId] = React.useState("");

  const [appSelected, setAppSelected] = React.useState("No app selected");

  const [worldSelected, setWorldSelected] = React.useState("No world selected");

  React.useEffect(() => {
    // Call the Python script or API to retrieve the new value
    // and update the context variable
    fetch('http://127.0.0.1:8000/exercises/exercise/get_project_url/')
      .then(response => response.text())
      .then(data => {
        setProjectURL(data);
        console.log("Se acaba de hacer");
      })
      .catch(error => {
        console.error(error);
      });
  }, []);

  const [visualization, setVisualization] = useState({
    specific: false,
    gazebo: false,
    console: false,
  });

  const [filename, setFileName] = useState("filename");
  const [codeChanged, setCodeChanged] = useState(false);
  const [editorCode, setEditorCode] = useState(`from GUI import GUI
from HAL import HAL
# Enter sequential code!

while True:
    # Enter iterative code!`);
  const [linterMessage, setLinterMessage] = useState([]);
  const editorCodeChange = (e) => {
    setEditorCode(e);
    userCodeChanged(true);
  };

  const userCodeChanged = (e) =>{
    setCodeChanged(e);
  }

  const changeSelectedNodeId = (e) =>{
    setSelectedNodeId(e);
  }

  const changeAppSelected = (e) => {
    setAppSelected(e);
  }

  const changeWorldSelected = (e) => {
    setWorldSelected(e);
  }

  const loadFileButton = (event) => {
    event.preventDefault();
    var fr = new FileReader();
    fr.onload = () => {
      setEditorCode(fr.result);
    };
    fr.readAsText(event.target.files[0]);
  };

  const loadInDirectory = (event) => {
    event.preventDefault();
    var fr = new FileReader();
    fr.onload = () => {
      fetch(
        'http://127.0.0.1:8000/exercises/exercise/load_file_into_directory/', {
        method: 'POST',
        headers: {
        'Content-Type': 'application/json',
        'X-CSRFToken': csrftoken
        },
        body: JSON.stringify({ body: fr.result, name: name})
        })
        .then((response) => response.text())
        .then((result) => {
            console.log(result);
            userCodeChanged(true);
        })
        .catch((error) => {
        console.error(error);
      });
    };
    fr.readAsText(event.target.files[0]);
    var name = event.target.files[0].name;
  };

  const copyInDirectory = (event) => {
    event.preventDefault();
    fetch(
    'http://127.0.0.1:8000/exercises/exercise/copy_file_into_directory/', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
            'X-CSRFToken': csrftoken
    },
        body: JSON.stringify({ path: selectedNodeId})
    })
    .then((response) => response.text())
    .then((result) => {
        console.log(result);
        userCodeChanged(true);
    })
    .catch((error) => {
        console.error(error);
    });
  };

  const deleteInDirectory = (event) => {
    event.preventDefault();
    fetch(
    'http://127.0.0.1:8000/exercises/exercise/delete_file_into_directory/', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
            'X-CSRFToken': csrftoken
    },
        body: JSON.stringify({ path: selectedNodeId })
    })
    .then((response) => response.text())
    .then((result) => {
        console.log(result);
        userCodeChanged(true);
        setEditorCode(`from GUI import GUI
from HAL import HAL
# Enter sequential code!

while True:
    # Enter iterative code!`);
    })
    .catch((error) => {
        console.error(error);
    });
  };

  const saveFileButton = () => {
    saveCode(filename, editorCode);
  };

  const handleFilename = (e) => {
    setFileName(e.target.value);
  };

  const changeVisualization = (visual) => {
    setVisualization(visual);
  };

  const toggleDisplayVisual = (value) => {
    setDisplayVisual(value);
  };

  const toggleDisplayEditor = (value) => {
    setDisplayEditor(value);
  }

  const toggleDisplayDirectory = (value) => {
    setDisplayDirectory(value);
  }

  const toggleChangeBroads = (value) => {
    setBroads(value);
  }

  const changeProjectURL = (value) => {
    setProjectURL(value);
  }

  const launch_file = (name) =>{
    return `<?xml version="1.0" encoding="UTF-8"?>
    <launch>
      <!-- We resume the logic in empty_world.launch, changing only the name of the world to be launched -->
      <include file="$(find gazebo_ros)/launch/empty_world.launch">
        <arg name="world_name" value="${name}.world"/> <!-- Note: the world_name is with respect to GAZEBO_RESOURCE_PATH environmental variable -->
        <arg name="paused" value="false"/>
        <arg name="use_sim_time" value="true"/>
        <arg name="gui" value="false"/>
        <arg name="headless" value="true"/>
        <arg name="debug" value="false"/>
        <arg name="verbose" default="true"/>
      </include>
    </launch>`;
  };

  const changeConfig = (world, name) => {

    const config = JSON.parse(
      document.getElementById("exercise-config").textContent
    );
    //config.application.params = { world: worldPath };
    config.launch[
      "0"
    ].launch_file = '/workspace/worlds/general_launch_file.launch';

    var zip = new JSZip();
    zip.file("general_launch_file.launch", launch_file(name));
    zip.file(`${name}.world`, world);
    zip.generateAsync({type:"base64"})
      .then(function(content) {
        config.launch_files = content;
      });
    console.log(config);
    return config;
  };



  const handleWorldChange = (world, name) => {
    const config = changeConfig(world, name);
    window.RoboticsExerciseComponents.commsManager.terminate().then(() => {
      window.RoboticsExerciseComponents.commsManager.launch(config);
    });
  };


  const [loading, setLoading] = useState(false);
  const [entryPoint, setEntryPoint] = useState("main.py");

  const changeEntryPoint = (value) => {
    setEntryPoint(value);
  }


  const loadCode = () => {
    fetch('http://127.0.0.1:8000/exercises/exercise/save_data/', {
            method: 'POST',
            headers: {
              'Content-Type': 'application/json',
              'X-CSRFToken': csrftoken
            },
            body: JSON.stringify({ data: editorCode })
    })
    setLoading(true);
    var zip = new JSZip();
    // TODO send all user code files
    // TODO set entrypoint the selected file to execute
    fetch('http://127.0.0.1:8000/exercises/exercise/get_files_content/' + projectURL + "/application")
      .then(response => response.json())
      .then(data => {
        console.log(data);
        console.log(entryPoint);
        data.files.forEach(file =>{
            zip.file(file[0], file[1]);
        });
        
        zip.generateAsync({type:"base64"})
            .then(function(content) {
              window.RoboticsExerciseComponents.commsManager
              .send("load", {
                zip: content,
                entrypoint: entryPoint,
              })
              .then(() => {
                runCode();
                setLoading(false);
              })
              .catch((response) => {
                let linterMessage = JSON.stringify(response.data.message).split("\\n");
                setLinterMessage(linterMessage);
                setLoading(false);
              });
            });
      })
      .catch(error => {
        console.log(error);
      });
  };

  const runCode = () => {
    window.RoboticsExerciseComponents.commsManager
      .run()
      .then(() => {
        console.log("running");
      })
      .catch((response) => console.error(response));
  };

  const [flagDirectoryView, setFlagDirectoryView] = useState(false);

   const changeFlagDirectoryView = (value) => {
    setFlagDirectoryView(value);
  };

  return (
    <NewManagerExerciseContext.Provider
      value={{
        editorCodeChange,
        userCodeChanged,
        filename,
        setFileName,
        editorCode,
        codeChanged,
        loadFileButton,
        saveFileButton,
        handleFilename,
        visualization,
        changeVisualization,
        linterMessage,
        setLinterMessage,
        displayVisual,
        toggleDisplayVisual,
        displayEditor,
        toggleDisplayEditor,
        displayDirectory,
        toggleDisplayDirectory,
        broad,
        toggleChangeBroads,
        projectURL,
        changeProjectURL,
        loadInDirectory,
        selectedNodeId,
        changeSelectedNodeId,
        copyInDirectory,
        deleteInDirectory,
        handleWorldChange,
        loadCode,
        loading,
        changeEntryPoint,
        appSelected,
        changeAppSelected,
        flagDirectoryView,
        changeFlagDirectoryView,
        worldSelected,
        changeWorldSelected,
      }}
    >
      {children}
    </NewManagerExerciseContext.Provider>
  );
}

ExerciseProvider.propTypes = {
  children: PropTypes.node,
};

export default NewManagerExerciseContext;
