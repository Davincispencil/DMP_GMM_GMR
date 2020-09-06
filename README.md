# DMP_GMM_GMR
Gaussian Mixture Regression is often used as a trajectory encoding method. And it actually gets a really good estimation of trajectory from multiple demonstraion and also has a full covariance matrix for all variables. But for estimating a trajectory, it really demands a few demonstrations and can not be generalized, which means that we need to retrain our GMR model, even we only want a slight modification on the previous one.

Dynamic Movement Primitives is kind of simple and useful method for trajectory encoding and generalization. By using second-order spring-damp system and non-linear force function, a DMP model can present any non-linear trajectory with only one demonstration. Note that in this model, non-linear force function could be learned by LWR or LWPR.

Although it' s true that we are able to estimate a trajectory by only one demonstraion(through DMP), but there is the possibility that in this demonstration, we might have some inevitable errors from sensors or experts. And the combination of GMR and DMP can handle out this problem decently. 

Instead of using GMR to replace LWR algorithm in DMP, here we use GMR to estimate the trajectory from multiple demonstrations and then use this learned trajectory as demonstration for DMP, which could avoid the sensor/expert errors and retain the generalization ability simutaneously.
