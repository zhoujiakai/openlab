#########################################################################################
# Take the first 3600 data of the column where the missing value is located as          #
# a training set and use the AutoReg model to train and ultimately predict the missing  #
# value to fill in the missing value                                                    #   
#########################################################################################
import pandas as pd
import numpy as np
from statsmodels.tsa.ar_model import AutoReg

# Reading a CSV file containing missing values
data = pd.read_csv('./xinan_uncompleted_data.csv')

for index, row in data.iterrows():
    if row.isnull().any(): 
        missing_indexes = row.index[row.isnull()] 
        for col in missing_indexes:
            # Take the first 3600 data of the column where the missing value is located as the training set
            train_data = data.loc[index - 10000:index - 1, col]

            # Fitting an AutoReg model
            model = AutoReg(train_data, lags=10)
            model_fit = model.fit()

            # Predict missing values and fill them in
            predicted_value = model_fit.predict(start=len(train_data), end=len(train_data))
            data.loc[index, col] = round(predicted_value.iloc[0],3)

data.to_csv('./xinan_completed_data.csv', index=False)




