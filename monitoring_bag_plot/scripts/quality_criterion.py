import numpy
import math

numpy.seterr(divide='ignore', invalid='ignore')

class quality_criterion():

    def __init__(self, time_list, data_list,param):
        self.time_list_ = time_list
        self.data_list_ = data_list
        self.data_analys_param()
        self.result_list = []
        self.result = ''
        self.criterion_value_dict = {}
        if  len(data_list) == 0:
            return
        for name in param.keys():
            if name == 'linear_criterion':
                self.result_list.append(self.linear_criterion(param[name][0], param[name][1]))
            elif name == 'tendency_criterion':
                self.result_list.append(self.tendency_criterion(param[name][0], param[name][1],param[name][2], param[name][3],param[name][4], param[name][5],param[name][6], param[name][7]))
            elif name == 'value_criterion':
                self.result_list.append(self.value_criterion(param[name][0],param[name][1]))
            elif name == 'const_critertion':
                self.result_list.append(self.const_critertion(param[name][0], param[name][1]))
        if 'error' in self.result_list:
            self.result = 'error'
        elif 'warn' in self.result_list and 'error' not in self.result_list:
            self.result = 'warn'
        else:
            self.result = 'ok'

## analys param ,get std_dev , list of derivation , Max/Min of data
    def data_analys_param(self):
       ## print self.data_list_
       ## print self.time_list_
        numpy_derivation= numpy.diff(self.data_list_)/numpy.diff(self.time_list_)
        self.derivation = numpy_derivation.tolist()
        self.Max_of_data = max(self.data_list_)
        self.Min_of_data = min(self.data_list_)


## Judge if linear. Linear, when the std_dev of Correlation is smaller than allowed and
    def linear_criterion(self, Correlation_warn, Correlation_error):
        X = numpy.vstack((self.data_list_,self.time_list_))
        Correlation = numpy.corrcoef(X.astype(float))
        val = round(numpy.linalg.det(Correlation), 4)
        self.criterion_value_dict['linear_criterion_value'] = val
        if val > Correlation_error:
            return "error"
        if val > Correlation_warn:
            return "warn"
        return "ok"

    def tendency_criterion(self, gradient_median_error,
            max_median_error, mid_median_error, min_median_error,
            gradient_median_warn, max_median_warn, mid_median_warn, min_median_warn):
        ##gradient = numpy.arctan(self.derivation)*180/numpy.pi
        abs_gradient = map(abs, self.derivation)
        abs_gradient.sort()
        gradient_lang = len(abs_gradient)
        min_gradient_median = numpy.median(abs_gradient[0:int(round(gradient_lang*0.25))])
        ##print abs_gradient[0:int(round(gradient_lang*0.25))]
        mid_gradient_median = numpy.median(
            abs_gradient[int(round(gradient_lang*0.25)):int(round(gradient_lang*0.75))])
        ##print abs_gradient[int(round(gradient_lang*0.25)):int(round(gradient_lang*0.75))]
        max_gradient_median = numpy.median(abs_gradient[int(round(gradient_lang*0.75)):int(gradient_lang)])
        ##print abs_gradient[int(round(gradient_lang*0.75)):int(gradient_lang)]
        gradient_median = numpy.median(abs_gradient)

        self.criterion_value_dict['gradient_median:All/High/Mid/Low'] = str(
            round(gradient_median,2)) +'/' + str(round(max_gradient_median,2))+'/'+\
            str(round(mid_gradient_median,2))+'/'+str(round(min_gradient_median,2))

        if min_gradient_median > min_median_error:
            return "error"
        if max_gradient_median > max_median_error:
            return "error"
        if mid_gradient_median > mid_median_error:
            return "error"
        if gradient_median > gradient_median_error:
            return "error"
        if gradient_median > gradient_median_warn:
            return "warn"
        if max_gradient_median > max_median_warn:
            return "warn"
        if min_gradient_median > min_median_warn:
            return "warn"
        if mid_gradient_median > mid_median_warn:
            return "warn"
        return "ok"

## Judge the value range
    def value_criterion(self, max_warn,max_error):
        self.criterion_value_dict['value_criterion'] = round(self.Max_of_data,4)
        if self.Max_of_data > max_error:
            return "error"
        if self.Max_of_data > max_warn:
            return "warn"
        return "ok"

## Judge the const
    def const_critertion(self, const_std_dev_warn,const_std_dev_error):
        self.const_std_dev_error = 10**const_std_dev_error
        self.const_std_dev_warn = 10**const_std_dev_warn
        const_std_dev = numpy.std(self.data_list_)
        self.criterion_value_dict['const_critertion'] = round(const_std_dev,4)
        if const_std_dev > self.const_std_dev_error:
            return "error"
        if const_std_dev > self.const_std_dev_warn:
            return "warn"
        return "ok"

 ##########################test###############################
if __name__ == '__main__':
    q1 = quality_criterion([1,2,3,4,5,6],[1,4,3,10,5,6],{'linear_criterion':[0.1,0.4],'const_critertion':[0.1,0.4],'tendency_criterion':[0.1,0.4,0.1,0.4,0.1,0.4,0.1,0.4]})
    print q1.result
   ## q2 = quality_criterion([1,2,3,4,5,6],[0,1999,0,1999,0,1999],['linear_criterion'])

