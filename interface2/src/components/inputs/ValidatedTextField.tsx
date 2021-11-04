import React, { FC } from 'react';
import { ValidateFieldsError } from 'async-validator';

import { FormHelperText, TextField, TextFieldProps } from '@mui/material';

interface ValidatedFieldProps {
  fieldErrors?: ValidateFieldsError;
  name: string;
}

export type ValidatedTextFieldProps = ValidatedFieldProps & TextFieldProps;

const ValidatedTextField: FC<ValidatedTextFieldProps> = ({ fieldErrors, fullWidth = true, ...rest }) => {
  const errors = fieldErrors && fieldErrors[rest.name];

  const renderErrors = () => {
    return errors && errors.map((e, i) => (<FormHelperText key={i}>{e.message}</FormHelperText>));
  };

  return (
    <>
      <TextField
        fullWidth={fullWidth}
        error={!!errors}
        {...rest}
      />
      {renderErrors()}
    </>
  );
};

export default ValidatedTextField;
