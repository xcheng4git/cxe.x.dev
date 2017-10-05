#pragma once
#include "ConstraintProposal.h"
#include "ConstraintProposalVoter.h"

namespace slam
{
	namespace constraints
	{
		struct ConstraintProposalValidator
		{
		public:
			struct Stage
			{
			private:
				friend struct ConstraintProposalValidator;

				int Id;
				bool OnlyKeepBest;
				vo::DenseTracker::Config TrackingConfig;
				ConstraintProposalVoterVector Voters;

				Stage(int id);
			public:
				Stage& keepBest();

				Stage& keepAll();

				Stage& trackingConfig(const vo::DenseTracker::Config& cfg);

				Stage& addVoter(ConstraintProposalVoter* v);
			};

			typedef std::vector<Stage> StageVector;

			ConstraintProposalValidator();

			Stage& createStage(int id);

			void validate(ConstraintProposalVector& proposals, bool debug = false);

			void keepBest(ConstraintProposalVector& proposals);

		private:
			StageVector stages_;
			vo::DenseTracker tracker_;

			void validate(Stage& stage, ConstraintProposalVector& proposals, bool debug = false);

			void printVotingResults(const Stage& stage, const ConstraintProposalVector& proposals);
		};

		typedef boost::shared_ptr<ConstraintProposalValidator> ConstraintProposalValidatorPtr;
	}
}