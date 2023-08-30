import React, { useEffect, useState } from "react";

import MenuItem from "@mui/material/MenuItem";
import LanguageIcon from "@mui/icons-material/Language";

import { FormControl, InputLabel, Select, TextField, Input } from "@mui/material";

export default function WorldSelector(props) {

  const { projectURL, handleWorldChange, worldSelected, changeWorldSelected } = React.useContext(props.context);

  const csrftoken = document.querySelector('[name=csrfmiddlewaretoken]').value;

  const[isOpen, setIsOpen] = useState(false)

  const [disabled, setDisabled] = useState(true);

  useEffect(() => {
    const callback = (message) => {
      if (message.data.state === "ready") {
        setDisabled(false);
      } else {
        setDisabled(true);
      }
    };
    window.RoboticsExerciseComponents.commsManager.subscribe(
      [window.RoboticsExerciseComponents.commsManager.events.STATE_CHANGED],
      callback
    );

    return () => {
      window.RoboticsExerciseComponents.commsManager.unsubscribe(
        [window.RoboticsExerciseComponents.commsManager.events.STATE_CHANGED],
        callback
      );
    };
  }, []);

  function MenuItems({ item, setIsOpen }) {
  const handleClick = () => {
    fetch(
      'http://127.0.0.1:8000/exercises/exercise/get_file_content/' +
        filePath +
        item.path
    )
      .then((response) => response.text())
      .then((result) => {
        changeWorldSelected(item.name);
        handleWorldChange(result, item.name);
        setIsOpen(false); // close the menu
        fetch(
        'http://127.0.0.1:8000/exercises/exercise/save_metadata_file/', {
        method: 'POST',
        headers: {
        'Content-Type': 'application/json',
        'X-CSRFToken': csrftoken
        },
        body: JSON.stringify({ url: projectURL+"/metadata/", data: result, name: item.name})
        })
        .then((response) => response.text())
        .then((result) => {
            console.log(result);
        })
      })
      .catch((error) => {
        console.error(error);
      });
  };

  return <MenuItem onClick={handleClick} value={item.name}>{item.name}</MenuItem>;
}


  function MapItems(files){
    const items = []
    files.forEach(file => {
      const pathSegments = window.navigator.platform.startsWith('Linux') ? file.split('/') : file.split('\\');file.split('\\');
      const newItem = {
        name: pathSegments[1].slice(0, -6),
        path: pathSegments.slice(0).join('/'),
      };
      items.push(newItem);
      });
    return items;
  }

  const filePath = projectURL + "/world/"
  const [files, setFiles] = React.useState([]);

  React.useEffect(() => {
    fetch('http://127.0.0.1:8000/exercises/exercise/get_folder_content/' + filePath)
      .then(response => response.json())
      .then(data => {
        setFiles(data.files);
      })
      .catch(error => {
        console.error(error);
      });
  }, [isOpen]);

  const menuItems = MapItems(files);
  return (
      <>
        <FormControl>
          <InputLabel id={"world-selector-label"}>
            World
            <LanguageIcon />
          </InputLabel>
          <Select
            disabled={disabled}
            labelId="world-selector-label"
            id={"world-selector"}
            label={"World"}
            defaultValue={"None"}
            MenuProps={{
              onClose: () => setIsOpen(false),
            }}
            open={isOpen}
            onOpen={() => setIsOpen(true)}
          >
            <MenuItem selected disabled value="None"><em>{worldSelected}</em></MenuItem>
            {menuItems.map((item) => (
              <MenuItems key={item.name} item={item} setIsOpen={setIsOpen} />
            ))}
          </Select>
        </FormControl>
      </>
  );
}
