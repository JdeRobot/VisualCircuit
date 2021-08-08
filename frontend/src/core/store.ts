import React, { Dispatch, SetStateAction } from 'react';

/**
 * Global state of the application
 */
export interface IGlobalState {
    /**
     * Lock status of current model
     */
    locked: boolean;
    /**
     * Indicates whether a (package) block is opened.
     */
    showingPackage: boolean;
}

/**
 * Provider pattern for Global state.
 */
interface IGlobalStateContextProps {
    state: IGlobalState;
    setState: Dispatch<SetStateAction<IGlobalState>>
}

export const GlobalState = React.createContext<IGlobalStateContextProps | undefined>(undefined);
// Context exposing the global state
export const useGlobalState = () => React.useContext(GlobalState)!;
