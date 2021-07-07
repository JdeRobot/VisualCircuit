import React, { Dispatch, SetStateAction } from 'react';

export interface IGlobalState {
    locked: boolean;
    showingPackage: boolean;
}

interface IGlobalStateContextProps {
    state: IGlobalState;
    setState: Dispatch<SetStateAction<IGlobalState>>
}

export const GlobalState = React.createContext<IGlobalStateContextProps | undefined>(undefined);
export const useGlobalState = () => React.useContext(GlobalState)!;
