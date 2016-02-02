import numpy as np
from sklearn.pipeline import Pipeline, FeatureUnion
from sklearn.externals import six
from sklearn.utils.metaestimators import if_delegate_has_method

__all__ = ['ImagePipeline', 'FeatureUnion']


class ImagePipeline(Pipeline):
    def _pre_transform_xy(self, X, y=None, **fit_params):
        fit_params_steps = dict((step, {}) for step, _ in self.steps)

        for pname, pval in six.iteritems(fit_params):
            step, param = pname.split('__', 1)
            fit_params_steps[step][param] = pval
        Xt = X
        yt = y

        for name, transform in self.steps[:-1]:
            if hasattr(transform, "fit_transform"):
                out = transform.fit_transform(Xt, yt, **fit_params_steps[name])
            else:
                out = transform.fit(Xt, yt, **fit_params_steps[name]) \
                    .transform(Xt, yt)
            # Handle transforms that only return X, because X could be a
            # namedTuple, we have to check type explicitly
            if type(out) == tuple and len(out) == 2:
                Xt, yt = out
            else:
                Xt = out

        return Xt, yt, fit_params_steps[self.steps[-1][0]]

    @staticmethod
    def _transform_x(X, steps):
        Xt = X
        for name, transform in steps:
            out = transform.transform(Xt)
            if type(out) == tuple and len(out) == 2:
                Xt, _ = out
            else:
                Xt = out
        return Xt

    @staticmethod
    def _stack_ys(ys):
        if isinstance(ys, list):
            return np.hstack(ys)
        else:
            return ys

    def fit(self, X, y=None, **fit_params):
        """Fit all the transforms one after the other and transform the
        data, then fit the transformed data using the final estimator.

        Parameters
        ----------
        X : iterable
            Training data. Must fulfill input requirements of first step of the
            pipeline.
        y : iterable, default=None
            Training targets. Must fulfill label requirements for all steps of
            the pipeline.
        """
        Xt, yt, fit_params = self._pre_transform_xy(X, y, **fit_params)
        # Because FeatureTransformer doesn't change yt, it might be a list of
        # yt, thus we have to stack yt ourselves
        yt = self._stack_ys(yt)
        self.steps[-1][-1].fit(Xt, yt, **fit_params)
        return self

    def fit_transform(self, X, y=None, **fit_params):
        """Fit all the transforms one after the other and transform the
        data, then use fit_transform on transformed data using the final
        estimator.

        Parameters
        ----------
        X : iterable
            Training data. Must fulfill input requirements of first step of the
            pipeline.

        y : iterable, default=None
            Training targets. Must fulfill label requirements for all steps of
            the pipeline.
        """
        Xt, yt, fit_params = self._pre_transform_xy(X, y, **fit_params)
        if hasattr(self.steps[-1][-1], 'fit_transform'):
            return self.steps[-1][-1].fit_transform(Xt, yt, **fit_params)
        else:
            return self.steps[-1][-1].fit(Xt, yt, **fit_params). \
                transform(Xt, yt)

    @if_delegate_has_method(delegate='_final_estimator')
    def predict(self, X):
        """Applies transforms to the data, and the predict method of the
        final estimator. Valid only if the final estimator implements
        predict.

        Parameters
        ----------
        X : iterable
            Data to predict on. Must fulfill input requirements of first step of
            the pipeline.
        """
        Xt = self._transform_x(X, self.steps[:-1])
        return self.steps[-1][-1].predict(Xt)

    @if_delegate_has_method(delegate='_final_estimator')
    def fit_predict(self, X, y=None, **fit_params):
        """Applies fit_predict of last step in pipeline after transforms.

        Applies fit_transforms of a pipeline to the data, followed by the
        fit_predict method of the final estimator in the pipeline. Valid
        only if the final estimator implements fit_predict.

        Parameters
        ----------
        X : iterable
            Training data. Must fulfill input requirements of first step of
            the pipeline.
        y : iterable, default=None
            Training targets. Must fulfill label requirements for all steps
            of the pipeline.
        """
        Xt, yt, fit_params = self._pre_transform(X, y, **fit_params)
        return self.steps[-1][-1].fit_predict(Xt, yt, **fit_params)

    @if_delegate_has_method(delegate='_final_estimator')
    def predict_proba(self, X):
        """Applies transforms to the data, and the predict_proba method of the
        final estimator. Valid only if the final estimator implements
        predict_proba.

        Parameters
        ----------
        X : iterable
            Data to predict on. Must fulfill input requirements of first step of
            the pipeline.
        """
        Xt = self._transform_x(X, self.steps[:-1])
        return self.steps[-1][-1].predict_proba(Xt)

    @if_delegate_has_method(delegate='_final_estimator')
    def decision_function(self, X):
        """Applies transforms to the data, and the decision_function method of
        the final estimator. Valid only if the final estimator implements
        decision_function.

        Parameters
        ----------
        X : iterable
            Data to predict on. Must fulfill input requirements of first step of
            the pipeline.
        """
        Xt = self._transform_x(X, self.steps[:-1])
        return self.steps[-1][-1].decision_function(Xt)

    @if_delegate_has_method(delegate='_final_estimator')
    def predict_log_proba(self, X):
        """Applies transforms to the data, and the predict_log_proba method of
        the final estimator. Valid only if the final estimator implements
        predict_log_proba.

        Parameters
        ----------
        X : iterable
            Data to predict on. Must fulfill input requirements of first step of
            the pipeline.
        """
        Xt = self._transform_x(X, self.steps[:-1])
        return self.steps[-1][-1].predict_log_proba(Xt)

    @if_delegate_has_method(delegate='_final_estimator')
    def transform(self, X, y=None):
        """Applies transforms to the data, and the transform method of the
        final estimator. Valid only if the final estimator implements
        transform.

        Parameters
        ----------
        X : iterable
            Data to predict on. Must fulfill input requirements of first step of
            the pipeline.
        """
        Xt = self._transform_x(X, self.steps)
        return Xt

    @if_delegate_has_method(delegate='_final_estimator')
    def score(self, X, y=None):
        """Applies transforms to the data, and the score method of the
        final estimator. Valid only if the final estimator implements
        score.

        Parameters
        ----------
        X : iterable
            Data to score. Must fulfill input requirements of first step of the
            pipeline.

        y : iterable, default=None
            Targets used for scoring. Must fulfill label requirements for all
            steps of the pipeline.
        """
        Xt = self._transform_x(X, self.steps[:-1])
        return self.steps[-1][-1].score(Xt, y)
