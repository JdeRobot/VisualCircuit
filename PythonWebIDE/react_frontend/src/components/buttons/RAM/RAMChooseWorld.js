import React, { useEffect, useState } from "react";

import MenuItem from "@mui/material/MenuItem";
import SettingsIcon from "@mui/icons-material/Settings";

import { FormControl, InputLabel, Select } from "@mui/material";

import worlds from "./../../../worlds.json";

export default function WorldSelector(props) {

  const { projectURL, changeFlagDirectoryView} = React.useContext(props.context);

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

  const handleClick = (menuItemValue, menuItemName) => {
    fetch('http://127.0.0.1:8000/exercises/exercise/get_folder_content/' + projectURL + "/world")
      .then(response => response.json())
      .then(data => {
        if (JSON.stringify(data).includes(menuItemName)){
          console.log("This world is already in your project");
          setIsOpen(false); // close the menu
        } else {
          fetch(
            'http://127.0.0.1:8000/exercises/exercise/add_world_file/', {
            method: 'POST',
            headers: {
              'Content-Type': 'application/json',
              'X-CSRFToken': csrftoken
            },
            body: JSON.stringify({ value: menuItemValue, url: projectURL + "/world/", name: menuItemName })
          })
            .then((response) => response.text())
            .then((result) => {
              console.log(result);
              changeFlagDirectoryView(true);
              setIsOpen(false); // close the menu
            })
        }
      })
      .catch((error) => {
        console.error(error);
      });
  };

  return (
      <>
        <FormControl>
          <InputLabel id={"world-selector-label"}>
            Add a World to your Project
            <SettingsIcon />
          </InputLabel>
          <Select
            disabled={disabled}
            defaultValue={"Follow_Line_Default"}
            labelId="world-selector-label"
            id={"world-selector"}
            label={"World"}
            MenuProps={{
              onClose: () => setIsOpen(false),
            }}
            open={isOpen}
            onOpen={() => setIsOpen(true)}
          >
            {worlds.map((menuItem) => (
            <MenuItem
              key={menuItem.name}
              onClick={() => handleClick(menuItem.contents, menuItem.id)}
              value={menuItem.name}
            >
              {menuItem.name}
            </MenuItem>
          ))}
          </Select>
        </FormControl>
      </>
  );
}
